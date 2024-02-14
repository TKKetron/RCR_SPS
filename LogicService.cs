using Iot.Device.Bmxx80;
using Iot.Device.Bno055;
using Microsoft.Extensions.Hosting;
using System;
using System.Collections.Generic;
using System.Device.Gpio;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Channels;
using System.Threading.Tasks;

namespace SPS2._0
{
    

    internal class LogicService : BackgroundService
    {

        private readonly IMotorService[] _motorServices;
        private readonly GpioController _gpioController;
        private readonly Bno055Sensor _bno055Sensor;
        private readonly Bmp280 _bmp280;
        private readonly SerialPort _gpsSerialPort;
        private double[] motorDir;

        public LogicService(IEnumerable<IMotorService> motorServices, GpioController gpioController, Bno055Sensor bno055Sensor, Bmp280 bmp280, SerialPort gpsSerialPort)
        {
            Console.WriteLine("logic service init");

            // Use the named instances to get references to MotorService instances
            _motorServices = new IMotorService[] { motorServices.FirstOrDefault(s => s.ID == 0),
                                                   motorServices.FirstOrDefault(s => s.ID == 1) };

            _gpioController = gpioController;
            _bno055Sensor = bno055Sensor;
            _bmp280 = bmp280;
            _gpsSerialPort = gpsSerialPort;

            motorDir = new double[Settings.PWM_CHANNELS.Count()];
            Console.WriteLine("Logic service done init");
        }
        protected override async Task ExecuteAsync(CancellationToken cancellationToken)
        {
            Task deployed = _gpioController.WaitForEventAsync(Settings.DEPLOY_DETECT, PinEventTypes.Falling, cancellationToken).AsTask();

            do
            {
                //TODO talk to radio
                //TODO init all services


                // Subscribe to the DataReceived event of the serial port
                _gpsSerialPort.DataReceived += GpsSerialPort_DataReceived;

                // Start reading data from the serial port
                _gpsSerialPort.Open();
            } while (!deployed.IsCompleted) ;

            while (!cancellationToken.IsCancellationRequested)
            {
                Task<List<Task>> motorsCalibrated = CalibrateMotors(cancellationToken);
                Task<double> newHeading = GetHeadingAsync();//TODO

                //TODO get any other data needed here

                List<Task> lSwitches = await motorsCalibrated;
                while (!Task.WhenAny(lSwitches).IsCompleted)
                {
                    double heading, ratio, PIDOUT;

                    //PID can start immediatly based on current pos and error
                    Task<double> PIDTask = CalculatePIDPowerAsync(); //TODO

                    //TODO await all needed data for loop
                    heading = await newHeading;

                    double[] mag = new double[2];
                    mag[1] = ((Math.Cos(heading) / Math.Cos(motorDir[0])) - (Math.Sin(heading) / Math.Sin(motorDir[0]))) * ((-Math.Cos(motorDir[0]) * Math.Sin(motorDir[0])) / Math.Sin(motorDir[1] - motorDir[0]));

                    mag[0] = Math.Sqrt(1 - (mag[1] * mag[1]));

                    ratio = mag[0] / mag[1];
                    PIDOUT = await PIDTask;

                    if (mag[0] > mag[1])
                    {
                        mag[0] = PIDOUT; //output of PID between 0-1 (this represents % of full speed, so 1 is full speed)
                        mag[1] = PIDOUT / ratio;
                    }
                    else
                    {
                        mag[1] = PIDOUT;
                        mag[0] = PIDOUT * ratio;
                    }

                    //TODO: double check my math

                    if (Math.Abs((motorDir[0] - heading) % (Math.PI * 2)) < Math.PI / 2)
                        mag[0] *= -1;

                    //update the motors target position
                    for (int i = 0; i < 2; i++)
                    {
                        _motorServices[i].SetTargetMotorPositionAsync(mag[i]);
                    }

                    newHeading = GetHeadingAsync();
                    //TODO get other data as needed


                    //TODO make packets to send and save as needed
                }

                
                int limitMotor = lSwitches.IndexOf(Task.WhenAny(lSwitches).Result);
                Console.WriteLine($"motor {limitMotor} limit switch hit");

            }
        }

        private Task<double> CalculatePIDPowerAsync() //TODO
        {
            return Task.FromResult(1.0);
        }

        private Task<double> GetHeadingAsync() //TODO
        {
            return Task.FromResult(0.0%(Math.PI*2));
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

            Console.WriteLine("turn motor 1 to 0");
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
            await Task.Delay(Settings.DIR_TIME);
            //TODO get GPs

            motorDir[ID] = 0%(2*Math.PI); //set motor dir after IMU filter

            await _motorServices[ID].SetTargetMotorPositionAsync(0, true);

        }

        private void GpsSerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // Read the available data from the serial port
            string data = _gpsSerialPort.ReadExisting();

            // Process the received data
            // Here, you need to implement the logic to parse the GPS coordinates from the received data
            // The format of the data and the parsing algorithm depends on the GPS module and the protocol it uses

            // Example: Parse the GPS coordinates from a NMEA sentence
            if (data.StartsWith("$GPGGA"))
            {
                string[] parts = data.Split(',');

                // Extract latitude and longitude
                string latitude = parts[2];
                string longitude = parts[4];

                // Process the latitude and longitude values as needed
                // ...

                Console.WriteLine($"Latitude: {latitude}, Longitude: {longitude}");
            }
        }

    }
}
