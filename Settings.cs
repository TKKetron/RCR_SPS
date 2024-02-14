namespace SPS2._0
{
    internal static class Settings
    {


        public static readonly int[] PWM_CHANNELS = { 0, 1 }; //pwm channels for each motor


        
        public static readonly int[][] LIMIT_SWITCHES = { new int[] { 5, 6 }, new int[] { 16, 26 } }; //limit switch pins (each array is for 1 motor)
        //public static readonly int[][] ENCODER_CHANNELS = { new int[] { 17, 27 }, new int[] { 23, 24 } }; //pins for encoder (each array is 1 motor)

        //KEYBOARD
        //public static readonly int[][] LIMIT_SWITCHES = { new int[] { 65, 83 }, new int[] { 90, 88 } }; //A and S for L1 and Z and X for L2
        //public static readonly int[][] ENCODER_CHANNELS = { new int[] { 72, 74 }, new int[] { 78, 77 } }; //H and J for E1, N and M for E2

        //Antenna switch
        //public const int ANTENNA_SWITCH_A = 22;
        //public const int ANTENNA_SWITCH_B = 4;

        //Deployment detection
        public const int DEPLOY_DETECT = 25;

        //Avionics Coms
        public static readonly int[] AVIONIC_COMS = { 18, 19, 20, 21 };


        public const double SMOOTH_AMOUNT = .0025; //amount motor speed is changed by at 1 given moment
        public const int SMOOTH_DELAY = 10;   //miliseconds before the motor speed is changed

        public const int MOTOR_FREQ = 50; //frew the pwm channel for motor is set to

        public static readonly double[] MOTOR_FULL_DIR = { 0.095, 0.055 }; //full forward and reverse duty cycle
        public const double MOTOR_STOP = 0.075; // stopped duty cycle

        public const double BUFFER = .9; //what percent of the actual range do we want to use
        public const int ERROR = 50; //how much error we will accept in the movement of the motor (50 would make a deadp space of 100 units (50 each side) where we would be stopped)

        public const int DIR_TIME = 30000; //time to wait to test the dir of the motor
        public const int BACK_OFF_TIME = 1000; // how long to back off a motor from limit sitch

        public const int ENCODER_PULL_RATE = 1000;


    }
}
