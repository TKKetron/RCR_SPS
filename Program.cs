using Iot.Device.Bmxx80;
using Iot.Device.Bno055;
using Iot.Device.Board;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using System.Device.Gpio;
using System.Device.I2c;
using System.IO.Ports;

/*
 * To change between pi testing and PC testing: 4 steps
 * 
 * STEP 1: THIS FILE (GPIO controller
 * GPIO board on pi,
 * Gerneric board on PC
 * 
 * STEP 2: THIS FILE AGAIN (hosted service)
 * logicservice for flight test
 * Testservice for testing
 * 
 * STEP 3: Settings file
 * Keybaord pins for pc
 * pi pins for pi
 * 
 * STEP 4: Motor Service
 * un-comment 24 an 97 (decliration and init of hardware PWM) for pi and comment 25 andd 98
 * un-comment 25 an 98 (decliration and init of software PWM) for pc and comment 24 andd 97
*/

namespace SPS2._0
{
    internal class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello, World!");

            using IHost host = new HostBuilder()
                .ConfigureServices(services =>
                {

                    services.AddSingleton<GpioController>(provider =>
                    {

                        //Create a new GpioController instance
                        GpioController controller = new GpioController();

                        //GenericBoard b = new GenericBoard();
                        //GpioController controller = b.CreateGpioController();

                        // Open the GPIO pin for encoders

                        foreach (int[] i in Settings.LIMIT_SWITCHES)
                        {
                            foreach (int j in i)
                                controller.OpenPin(j, PinMode.Input);
                        }

                        controller.OpenPin(Settings.DEPLOY_DETECT, PinMode.Input, 1);



                        return controller;
                    });

                    // Register and resolve BNO055Sensor as Singleton
                    services.AddSingleton<Bno055Sensor>(s =>
                    {
                        I2cDevice i2cDevice = I2cDevice.Create(new I2cConnectionSettings(1, Bno055Sensor.DefaultI2cAddress));
                        return new Bno055Sensor(i2cDevice);
                    });

                    // Register and resolve Bmp280 as Singleton
                    services.AddSingleton<Bmp280>(s =>
                    {
                        I2cDevice i2cDevice = I2cDevice.Create(new I2cConnectionSettings(1, Bmp280.DefaultI2cAddress));
                        return new Bmp280(i2cDevice);
                    });

                    services.AddSingleton<SerialPort>(provider =>
                    {
                        string serialPortName = "/dev/ttyS0"; // Replace with the actual serial port name
                        int baudRate = 9600; // Replace with the appropriate baud rate
                        SerialPort serialPort = new SerialPort(serialPortName, baudRate);
                        serialPort.Open();
                        return serialPort;
                    });



                    // Register the services that use the GpioController
                    services.AddScoped<IMotorService>(s => new MotorService(0, s.GetService<GpioController>()));
                    services.AddScoped<IMotorService>(s => new MotorService(1, s.GetService<GpioController>()));

                    //Add main service
                    services.AddHostedService<LogicService>();
                    //services.AddHostedService<TestService>();


                })
            .Build();

            host.Run();
        }
    }
}