
package frc.team1918.robot;

public class Constants {

    public static final class Global {
        //Global Constants
        public final static int kTimeoutMs = 30; //Timeout for reporting in DS if action fails, set to 0 to skip confirmation
        public final static int PID_PRIMARY = 0;  //Talon PID slot for primary loop
        public final static int PID_TURN = 1; //Talon PID slot for auxillary loop
        public final static int ROBOT_WIDTH = 23; //Width of the robot frame
        public final static int ROBOT_LENGTH = 26; //Length of the robot frame
    }

    public static final class DriveTrain {
        //DriveTrain definitions
        public final static String DT_HOMES_FILE = "/home/lvuser/swerveHomes2021.txt"; //The file where we save the homes data to persist reboots
        public final static double DT_TURN_MULT_STATIONARY = 0.5; //Turn multiplier while not moving
        public final static double DT_TURN_MULT_MOVING = 0.9; //Turn multiplier while moving
        public final static boolean DT_TURN_MULT_BEFORE_DB = true; //Apply turn multiplier before deadband
        public final static double DT_HOME_DELAY = 0.75; //Seconds to wait for homing before reset encoders
        public final static int DT_HOME_MARGIN_OF_ERROR = 20; //Encoder ticks margin to consider home (plus or minus this amount)
        public final static boolean DT_USE_FIELD_CENTRIC = false; //Set to true to use field-centric drive
        public final static boolean DT_DRIVE_DISABLED = false; //Set to true to disable the drive motors (for lab)
        public final static double DT_TURN_P = 3.25;
        public final static double DT_TURN_I = 0.0;
        public final static double DT_TURN_D = 50.0;
        public final static double DT_TURN_IZONE = 0;

        public final static int DT_FL_DRIVE_MC_ID = 2; //Front Left Drive Motor Controller ID //SPARKMAX
        public final static int DT_FL_TURN_MC_ID = 7; //Front Left Turn Motor Controller ID //TALONSRX
        // public final static int DT_FL_HOME = 0; //Front Left encoder value at home position
        
        public final static int DT_FR_DRIVE_MC_ID = 16; //Front Right Drive Motor Controller ID //SPARKMAX
        public final static int DT_FR_TURN_MC_ID = 4; //Front Right Turn Motor Controller ID //TALONSRX
        // public final static int DT_FR_HOME = 0; //Front Right encoder value at home position

        public final static int DT_RL_DRIVE_MC_ID = 3; //Rear Left Drive Motor Controller ID //SPARKMAX
        public final static int DT_RL_TURN_MC_ID = 8; //Rear Left Turn Motor Controller ID //TALONSRX
        // public final static int DT_RL_HOME = 0; //Rear Left encoder value at home position
        
        public final static int DT_RR_DRIVE_MC_ID = 1; //Rear Right Drive Motor Controller ID //SPARKMAX
        public final static int DT_RR_TURN_MC_ID = 11; //Rear Right Turn Motor Controller ID //TALONSRX
        // public final static int DT_RR_HOME = 0; //Rear Right encoder value at home position
    }
    
    public static final class OI {
        /**
         * This class is based on 2 Logitech controllers, a driver and an operator, setup for swerve drive
         */
        public final static int OI_JOY_DRIVE = 0; //ID of Driver Joystick
        public final static int OI_JOY_OPER = 1; //ID of Operator Joystick
        public final static double OI_JOY_DEADBAND = 0.1; //Deadband for analog joystick axis
        // public static enum driveDpadDirection {UP,DOWN,IDLE};
        
        //Logitech Controller buttons  //In Java, buttons are 1-based array, BUT NOT AXIS
        private final static int LOGITECH_BTN_A = 1; //A Button
        private final static int LOGITECH_BTN_B = 2; //B Button
        private final static int LOGITECH_BTN_X = 3; //X Button
        private final static int LOGITECH_BTN_Y = 4; //Y Button
        private final static int LOGITECH_BTN_LB = 5; //Left Bumper (L1)
        private final static int LOGITECH_BTN_RB = 6; //Right Bumper (R1)
        private final static int LOGITECH_BTN_BACK = 7; //Back Button (Select)
        private final static int LOGITECH_BTN_START = 8; //Start Button
        private final static int LOGITECH_BTN_L = 9; //Left Stick Press (L3)
        private final static int LOGITECH_BTN_R = 10; //Right Stick Press (R3)
        private final static int LOGITECH_AXIS_LH = 0; //Left Analog Stick horizontal
        private final static int LOGITECH_AXIS_LV = 1; //Left Analog Stick vertical
        private final static int LOGITECH_AXIS_LT = 2; //Analog Left Trigger
        private final static int LOGITECH_AXIS_RT = 3; //Analog Right Trigger
        private final static int LOGITECH_AXIS_RH = 4; //Right Analog Stick horizontal
        private final static int LOGITECH_AXIS_RV = 5; //Right Analog Stick vertical
        private final static int LOGITECH_DPAD_UP = 0;
        private final static int LOGITECH_DPAD_UPRIGHT = 45;
        private final static int LOGITECH_DPAD_RIGHT = 90;
        private final static int LOGITECH_DPAD_DNRIGHT = 135;
        private final static int LOGITECH_DPAD_DN = 180;
        private final static int LOGITECH_DPAD_DNLEFT = 225;
        private final static int LOGITECH_DPAD_LEFT = 270;
        private final static int LOGITECH_DPAD_UPLEFT = 315;
        private final static int LOGITECH_DPAD_IDLE = -1; 
    
    //Temporary
        public final static int DRIVE_BTN_CALIBRATE_START = LOGITECH_BTN_B; //temporary for testing
        public final static int DRIVE_BTN_CALIBRATE_STOP = LOGITECH_BTN_A; //temporary for testing

    //Driver Controller
        public final static int DRIVE_AXIS_STRAFE = LOGITECH_AXIS_LH; //Axis that moves the robot side to side on the field
        public final static int DRIVE_AXIS_FWD = LOGITECH_AXIS_LV; //Axis that moves the robot up and down the field
        public final static int DRIVE_AXIS_TURN = LOGITECH_AXIS_RH; //Axis that controls the rotation of the robot
        public final static int DRIVE_BTN_ALLUP = LOGITECH_BTN_A; //Move collector to Up position
        public final static int DRIVE_BTN_ANTIGRAV = LOGITECH_BTN_B; //Engage anti-backdrive for climber
        public final static int DRIVE_BTN_MIXER_FEED = LOGITECH_BTN_X; //Run the mixer in the forward direction
        public final static int DRIVE_BTN_MIXER_FEEDSTUCK = LOGITECH_BTN_LB; //Reverse the mixer direction to unstick power cells
        public final static int DRIVE_BTN_HOMESWERVE = LOGITECH_BTN_Y; //Home the swerve modules
        public final static int DRIVE_BTN_ABSZERO_KEY1 = LOGITECH_BTN_BACK; //DRIVER KEY1 and OPER KEY2 are required for this
        public final static int DRIVE_DPAD_GYRO_RESET = LOGITECH_DPAD_LEFT;
        //Drive controller DPAD used as range selector for shooter speed (top half and bottom half)
        public final static int DRIVE_DPAD_THROTUP_UL = LOGITECH_DPAD_UPLEFT;
        public final static int DRIVE_DPAD_THROTUP_UP = LOGITECH_DPAD_UP;
        public final static int DRIVE_DPAD_THROTUP_UR = LOGITECH_DPAD_UPRIGHT;
        public final static int DRIVE_DPAD_THROTDN_DL = LOGITECH_DPAD_DNLEFT;
        public final static int DRIVE_DPAD_THROTDN_DN = LOGITECH_DPAD_DN;
        public final static int DRIVE_DPAD_THROTDN_DR = LOGITECH_DPAD_DNRIGHT;
    //Operator Controller        
        public final static int OPER_AXIS_CLIMB = LOGITECH_AXIS_LV; //Axis that controls the climber up and down
        public final static int OPER_AXIS_COLLECTOR_OUT = LOGITECH_AXIS_RT; //Axis that runs the collector out (actually a trigger button)
        public final static int OPER_BTN_SHOOT_WALL = LOGITECH_BTN_A; //Shoot from at the wall
        public final static int OPER_BTN_SHOOT_SHORT = LOGITECH_BTN_B; //Shoot from close to the wall
        public final static int OPER_BTN_SHOOT_LINE = LOGITECH_BTN_X; //Shoot from the initiation line
        public final static int OPER_BTN_SHOOT_TRENCH = LOGITECH_BTN_Y; //Shoot from the trench
        public final static int OPER_BTN_TOG_MIDDOWN = LOGITECH_BTN_LB; //Toggle collector arm between middle and down position
        public final static int OPER_BTN_COLLECTOR_IN = LOGITECH_BTN_RB; //Run the collector in
        public final static int OPER_BTN_ABSZERO_KEY2 = LOGITECH_BTN_BACK; //DRIVER KEY1 and OPER KEY2 are required for this
        public final static int OPER_DPAD_COLLECTOR_UP = LOGITECH_DPAD_LEFT; //Move collector to up
        public final static int OPER_DPAD_COLLECTOR_MID = LOGITECH_DPAD_RIGHT; //Move collector to middle
        //We get dpad values with joystick.getPOV(0);
    }
}

