using Microsoft.Extensions.Hosting;
using System;
using System.Collections.Generic;
using System.Device.Gpio;
using System.Linq;
using System.Text;
using System.Threading.Channels;
using System.Threading.Tasks;

namespace SPS2._0
{
    

    internal class TestService : BackgroundService
    {
        private readonly IMotorService[] _motorServices;
        private readonly Channel<LogicInput> _logicInputs;
        private readonly GpioController _gpioController;
        private double[] motorDir;

        private struct LogicInput
        {
            public int value;
            public LogicInputType type;
            public static LogicInput MotorError(int ID)
            {
                return new LogicInput
                {
                    value = ID,
                    type = LogicInputType.MotorError
                };
            }
        }

        enum LogicInputType
        {
            MotorError
        };

        public TestService(IEnumerable<IMotorService> motorServices, GpioController gpioController)
        {
            Console.WriteLine("logic service init");

            // Use the named instances to get references to MotorService instances
            _motorServices = new IMotorService[] { motorServices.FirstOrDefault(s => s.ID == 0), 
                                                   motorServices.FirstOrDefault(s => s.ID == 1) };

            if (_motorServices[0] == null)
                throw new Exception("motor 1 did not init");
            if (_motorServices[1] == null)
                throw new Exception("motor 2 did not init");
            _gpioController = gpioController;
            motorDir = new double[Settings.PWM_CHANNELS.Count()];
            _logicInputs = Channel.CreateUnbounded<LogicInput>();
            Console.WriteLine("Logic service done init");

        }

        protected override async Task ExecuteAsync(CancellationToken cancellationToken)
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                Task<List<Task>> motorsCalibrated = CalibrateMotors(cancellationToken);

                //TODO get any other data needed here

                List<Task> lSwitches = await motorsCalibrated;
                Console.WriteLine("Calibration Done Begin Tests:");
                while (!Task.WhenAny(lSwitches).IsCompleted)
                {
                    int m, w;
                    double pos;
                    Console.WriteLine("enter motor id: ");
                    m = Convert.ToInt32(Console.ReadLine());
                    Console.WriteLine("enter pos: ");
                    pos = Convert.ToDouble(Console.ReadLine());
                    Console.WriteLine("force wait (0 yes, 1 no): ");
                    w = Convert.ToInt32(Console.ReadLine());

                    Console.WriteLine($"turning motor id {m} to pos {pos} and wait status is {w}");
                    _motorServices[m].SetTargetMotorPositionAsync(pos, Convert.ToBoolean(w));


                }


                int limitMotor = lSwitches.IndexOf(Task.WhenAny(lSwitches).Result);
                Console.WriteLine($"motor {limitMotor} limit switch hit");

            }
        }


        private async Task<List<Task>> CalibrateMotors(CancellationToken cancellationToken)
        {
            Console.WriteLine("start motor 1");
            Task<TaskCompletionSource> motor1 = _motorServices[0].MotorInitAsync(cancellationToken);

            Console.WriteLine("start motor 2");
            Task<TaskCompletionSource> motor2 = _motorServices[1].MotorInitAsync(cancellationToken);

            await Task.WhenAll(motor1, motor2);
            Console.WriteLine("Both motors init");


            TaskCompletionSource m1Limit = motor1.Result;
            TaskCompletionSource m2Limit = motor2.Result;

            Console.WriteLine("turn motor id 1 to 0");
            await _motorServices[1].SetTargetMotorPositionAsync(0, true);

            await SetMotorDirAsync(0);
            await SetMotorDirAsync(1);

            List<Task> limitSwitchHit = new List<Task>
            { 
                m1Limit.Task, 
                m2Limit.Task 
            };

            return limitSwitchHit;

        }

        private async Task SetMotorDirAsync(int ID)
        {
            //TODO get GPS

            await _motorServices[ID].SetTargetMotorPositionAsync(1, true);
            Console.WriteLine($"delaying to get dir of motor id {ID}");
            await Task.Delay(Settings.DIR_TIME);
            //TODO get GPs

            motorDir[ID] = 0%(2*Math.PI); //set motor dir after IMU filter

            await _motorServices[ID].SetTargetMotorPositionAsync(0, true);

        }



    }
}
