using Microsoft.Extensions.Hosting;
using System.Device.Gpio;
using System.Device.Pwm;
using System.Device.Pwm.Drivers;
using System.Threading.Channels;

namespace SPS2._0
{



    public interface IMotorService
    {
        int ID { get; }
        Task SetTargetMotorPositionAsync(double targetMotorPosition, bool wait = false);
        Task<TaskCompletionSource> MotorInitAsync(CancellationToken cancellationToken);
        Task MotorStopAsync(CancellationToken cancellationToken);
    }

    internal class MotorService : BackgroundService, IMotorService
    {
        private readonly Channel<MotorInput> _motorInputs;
        private readonly GpioController _gpioController;
        private readonly PwmChannel motor;
        //private readonly SoftwarePwmChannel motor;
        int MAX_POS; //to be decided at startup to determine range of motor
        public int ID { get; init; }
        private int forward, backward;
        TaskCompletionSource limitSwitchHit;

        private struct MotorInput
        {
            public double value;
            public int value2;
            public MotorInputType type;
            public TaskCompletionSource atPosition;

            public static MotorInput TargetPositionChanged(double targetPosition)
            {
                return new MotorInput
                {
                    value = targetPosition,
                    type = MotorInputType.TargetPosition
                };
            }

            public static MotorInput TargetPositionChanged(double targetPosition, TaskCompletionSource atPosition)
            {
                return new MotorInput
                {
                    value = targetPosition,
                    type = MotorInputType.TargetPosition,
                    atPosition = atPosition
                };
            }

            public static MotorInput MotorPositionChanged(int currentPosition)
            {
                return new MotorInput
                {
                    value2 = currentPosition,
                    type = MotorInputType.CurrentPosition
                };
            }


            public static MotorInput StartController()
            {
                return new MotorInput
                {
                    value = 1,
                    type = MotorInputType.Start
                };
            }

            public static MotorInput StopController()
            {
                return new MotorInput
                {
                    type = MotorInputType.Stop
                };
            }
        }

        enum MotorInputType
        {
            CurrentPosition,
            TargetPosition,
            Start,
            Stop
        };

        public MotorService(int ID, GpioController gpioController)
        {
            Console.WriteLine(String.Format("Init motor {0}",ID));

            motor = PwmChannel.Create(0, Settings.PWM_CHANNELS[ID], Settings.MOTOR_FREQ, Settings.MOTOR_STOP);
            //motor = new SoftwarePwmChannel(Settings.PWM_CHANNELS[ID], Settings.MOTOR_FREQ, Settings.MOTOR_STOP, false, gpioController);
            _motorInputs = Channel.CreateUnbounded<MotorInput>();
            _gpioController = gpioController;
            this.ID = ID;
        }



        protected override async Task ExecuteAsync(CancellationToken cancellationToken)
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                await foreach (MotorInput input in _motorInputs.Reader.ReadAllAsync(cancellationToken))
                {
                    if (input.type == MotorInputType.Start)
                        break;
                }
                motor.Start();
                Console.WriteLine("motor fully started");
                using (CancellationTokenSource loopCancellationTokenSource = new CancellationTokenSource())
                {
                    using (CancellationTokenSource linkedCancellationToken = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, loopCancellationTokenSource.Token))
                    {
                        Task limitSwitchTask = ListenToLimitSwitchAsync(linkedCancellationToken.Token);
                        Task encoderTask = ListenToEncoderAsync(linkedCancellationToken.Token);
                        Task inputTask = ListenForMotorInputsAsync(linkedCancellationToken.Token);

                        Task completedTask = await Task.WhenAny(limitSwitchTask, encoderTask, inputTask);
                        if (completedTask == limitSwitchTask)
                        {
                            // Limit switch triggered, cancel all other tasks and restart loop
                            loopCancellationTokenSource.Cancel();
                            motor.Start();
                            await Task.Delay(1000); //back the motor off the switch
                            motor.Stop();
                            await Task.WhenAll(encoderTask, inputTask);
                            limitSwitchHit.SetResult();
                            continue;
                        }
                        else if (completedTask == inputTask)
                        {
                            loopCancellationTokenSource.Cancel();
                            await Task.WhenAll(encoderTask, limitSwitchTask);
                            continue;
                        }
                        {
                            // Logic Service Ended the motor runtime
                            linkedCancellationToken.Cancel();
                            await Task.WhenAll(encoderTask, inputTask, limitSwitchTask);
                            break;
                        }
                    }
                }
            }
        }



        /// <summary>
        /// DI function to allow the LogicService to change the motor postition
        /// </summary>
        /// <param name="targetMotorPosition"></param>
        /// <returns></returns>
        public async Task SetTargetMotorPositionAsync(double targetMotorPosition, bool wait = false)
        {
            Console.WriteLine("motor change called");
            if (!wait)
            {
                await _motorInputs.Writer.WriteAsync(MotorInput.TargetPositionChanged(targetMotorPosition)).AsTask();
                return;
            }
            TaskCompletionSource atPosition = new TaskCompletionSource();
            await _motorInputs.Writer.WriteAsync(MotorInput.TargetPositionChanged(targetMotorPosition, atPosition)).AsTask();
            await atPosition.Task;
        }



        /// <summary>
        /// listens to limit switches to automatically stop the PWM channel the motor is on, and writes to the _motorInput channel to re calibrate
        /// </summary>
        /// <param name="cancellationToken"></param>
        /// <returns></returns>
        public async Task ListenToLimitSwitchAsync(CancellationToken cancellationToken)
        {
            Console.WriteLine($"listening to limit on {ID}");

            List<Task> l = new List<Task>
            {
                _gpioController.WaitForEventAsync(Settings.LIMIT_SWITCHES[ID][0], PinEventTypes.Rising, cancellationToken).AsTask(),
                _gpioController.WaitForEventAsync(Settings.LIMIT_SWITCHES[ID][1], PinEventTypes.Rising, cancellationToken).AsTask()
            };

            await Task.WhenAny(l);
            Console.WriteLine($"ERROR ERROR Limit on motor id: {ID} hit");

            motor.Stop();


            if (motor.DutyCycle == Settings.MOTOR_FULL_DIR[0])
                motor.DutyCycle = Settings.MOTOR_FULL_DIR[1];
            else
                motor.DutyCycle = Settings.MOTOR_FULL_DIR[0];



        }

        /// <summary>
        /// Listens for either the A or B channel encoder to go high, then updates the position based on which goes high first. then waits for either the other to go high or the same to go high to restart the function.
        /// </summary>
        /// <param name="cancellationToken"></param>
        /// <returns></returns>
        private async Task ListenToEncoderAsync(CancellationToken cancellationToken)
        {
            Console.WriteLine($"listening to limit switch on motor id {ID}");

            int currentPosition = (int)(MAX_POS/Settings.BUFFER);

            //fix max pos being negative AFTER you set the position since you owuld be at the negative end
            if (MAX_POS < 0)
                MAX_POS *= -1;

            while (!cancellationToken.IsCancellationRequested)
            {
                //TODO oull encoder DATA
                await Task.Delay(Settings.ENCODER_PULL_RATE);

                await _motorInputs.Writer.WriteAsync(MotorInput.MotorPositionChanged(currentPosition));


            }
        }
        
        /// <summary>
        /// Listens to the _motorInputs channel for data to be written about how to update the motor
        /// </summary>
        /// <param name="cancellationToken"></param>
        /// <returns></returns>
        private async Task ListenForMotorInputsAsync(CancellationToken cancellationToken)
        {
            int currentMotorPosition = 0;
            double targetMotorPosition = 0;
            TaskCompletionSource? currentCompletionTask = null;

            await foreach (MotorInput input in _motorInputs.Reader.ReadAllAsync(cancellationToken))
            {
                switch (input.type)
                {
                    case MotorInputType.CurrentPosition:
                        Console.WriteLine($"cur pos update recieved on motor {ID}");
                        currentMotorPosition = input.value2;
                        break;

                    case MotorInputType.TargetPosition:
                        if (currentCompletionTask != null)
                        {
                            Console.WriteLine($"new pos recieved but currently waiting for old pos to be reached so ignoring on motor id {ID}");
                            continue;
                        }
                        if (input.atPosition != null)
                            currentCompletionTask = input.atPosition;
                        targetMotorPosition = input.value;
                        Console.WriteLine($"new target pos recieved on motor id {ID}");

                        break;
                    case MotorInputType.Stop:
                        Console.WriteLine($"stopping motor id {ID} something wrong?");
                        return;
                }

                Console.WriteLine($"updating motor id {ID} to go to pos {targetMotorPosition}");


                if (currentMotorPosition < targetMotorPosition * MAX_POS + Settings.ERROR && currentMotorPosition > targetMotorPosition * MAX_POS - Settings.ERROR)
                {
                    motor.DutyCycle = Settings.MOTOR_STOP;
                    if (currentCompletionTask != null)
                    {
                        currentCompletionTask.SetResult(); //AT TARGET POS
                        currentCompletionTask = null;
                    }
                }
                else if (currentMotorPosition < targetMotorPosition * MAX_POS)
                {
                    while (motor.DutyCycle != Settings.MOTOR_FULL_DIR[forward])
                    {
                        if (motor.DutyCycle < Settings.MOTOR_FULL_DIR[forward])
                            motor.DutyCycle += Settings.SMOOTH_AMOUNT;
                        else
                            motor.DutyCycle -= Settings.SMOOTH_AMOUNT;
                        await Task.Delay(Settings.SMOOTH_DELAY);
                    }
                }
                else
                {
                    while (motor.DutyCycle != Settings.MOTOR_FULL_DIR[backward])
                    {
                        if(motor.DutyCycle < Settings.MOTOR_FULL_DIR[backward])
                            motor.DutyCycle += Settings.SMOOTH_AMOUNT;
                        else
                            motor.DutyCycle -= Settings.SMOOTH_AMOUNT;
                        await Task.Delay(Settings.SMOOTH_DELAY);
                    }
                }
            }
        }

        //Calibration Code
        #region Calibration

        /// <summary>
        /// Calibrates the motor by turning it full one direction, then starts reading from encoders, then full one direction, then sets the max position of the encoders and leaves motor full forward (1)
        /// </summary>
        /// <param name="cancellationToken"></param>
        /// <returns></returns>
        public async Task<TaskCompletionSource> MotorInitAsync(CancellationToken cancellationToken)
        {
            Console.WriteLine("Starting Calibration");


            motor.DutyCycle = Settings.MOTOR_FULL_DIR[1]; //full reverse till you hit back limit switch
            limitSwitchHit = new TaskCompletionSource();

            motor.Start();
            Console.WriteLine("Waiting for limit switch");

            List<Task> switches = new List<Task>
            {
                _gpioController.WaitForEventAsync(Settings.LIMIT_SWITCHES[ID][0], PinEventTypes.Rising, cancellationToken).AsTask(),
                _gpioController.WaitForEventAsync(Settings.LIMIT_SWITCHES[ID][1], PinEventTypes.Rising, cancellationToken).AsTask()
            };

            Task switchP = await Task.WhenAny(switches);
            motor.Stop();
            Task<int> encoder;
            if (switches.IndexOf(switchP) == 1)
                encoder = CalibrateEncodersAsync(0, cancellationToken);//start reading encoders
            else
                encoder = CalibrateEncodersAsync(1, cancellationToken);
            motor.DutyCycle = Settings.MOTOR_FULL_DIR[0]; //go full forward
            motor.Start();

            MAX_POS = (int)(await encoder / 2.0 * Settings.BUFFER); //the limit siwtch in encoer will stop motor, then we set max pos
            motor.DutyCycle = Settings.MOTOR_STOP; //change the pwm to stop

            if (MAX_POS < 0)
            {
                forward = 1;
                backward = 0;
            }
            else
            {
                forward = 0;
                backward = 1;
            }


            await _motorInputs.Writer.WriteAsync(MotorInput.StartController()); //tell the motor to start

            return limitSwitchHit;

        }


        /// <summary>
        /// Returns the value of the encoder from the position the motor was in when called to the full position of whatever direction it travels in.
        /// </summary>
        /// <param name="cancellationToken"></param>
        /// <returns></returns>
        private async Task<int> CalibrateEncodersAsync(int switchC, CancellationToken cancellationToken)
        {
            int currentPosition = 0;

            Console.WriteLine("waiting for limit switch again");


            await _gpioController.WaitForEventAsync(Settings.LIMIT_SWITCHES[ID][switchC], PinEventTypes.Rising, cancellationToken).AsTask();
                             


            motor.Stop();

            return currentPosition;//return ICMD
        }

        public async Task MotorStopAsync(CancellationToken cancellationToken)
        => await _motorInputs.Writer.WriteAsync(MotorInput.StopController());


        #endregion

    }
}
