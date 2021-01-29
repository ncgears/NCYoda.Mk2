//OI = Operator Interface
package frc.team1918.robot;

import frc.team1918.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class Helpers {
    public static final class OI {
        private static Joystick dj = new Joystick(Constants.OI.OI_JOY_DRIVE);
        private static Joystick oj = new Joystick(Constants.OI.OI_JOY_OPER);
        //DRIVER CONTROLS
        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output 
         * @return double precision -1 to 1 of the strafe axis 
         */
        public final static double getAxisStrafeValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.DRIVE_AXIS_STRAFE)) : dj.getRawAxis(Constants.OI.DRIVE_AXIS_STRAFE);
        }

        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the fwd axis 
         */
        public final static double getAxisFwdValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.DRIVE_AXIS_FWD)*-1) : dj.getRawAxis(Constants.OI.DRIVE_AXIS_FWD)*-1;
        }

        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the turn axis 
         */
        public final static double getAxisTurnValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(dj.getRawAxis(Constants.OI.DRIVE_AXIS_TURN)) : dj.getRawAxis(Constants.OI.DRIVE_AXIS_TURN);
        }
        /**
         * @return integer value of Drive DPAD
         */
        public int getPOVDrive() {
            return dj.getPOV(0);
        }
        /**
         * This function determines if we are pressing any of the top half of the driver DPAD.
         * @return boolean indicating top half of Driver DPAD is pressed
         */
        public boolean isDriveDpadUp() {
            switch (getPOVDrive()) {
                case Constants.OI.DRIVE_DPAD_UPLEFT:
                case Constants.OI.DRIVE_DPAD_UP:
                case Constants.OI.DRIVE_DPAD_UPRIGHT: return true;
            }
            return false;
        }
        /**
         * This function determines if we are pressing any of the bottom half of the driver DPAD.
         * @return boolean indicating bottom half of Drive DPAD is pressed
         */
        public boolean isDriveDpadDown() {
            switch (getPOVDrive()) {
                case Constants.OI.DRIVE_DPAD_DNLEFT:
                case Constants.OI.DRIVE_DPAD_DN:
                case Constants.OI.DRIVE_DPAD_DNRIGHT: return true;
            }
            return false;
        }
        /**
         * This function returns the direction of the Drive DPAD
         * @return enum OI.driveDpadDirection containing one of UP, DOWN, IDLE
         */
        public Constants.OI.driveDpadDirection getDriveDpadDirection() {
            switch (getPOVDrive()) {
                //Top half of Drive DPAD
                case Constants.OI.DRIVE_DPAD_UPLEFT:
                case Constants.OI.DRIVE_DPAD_UP:
                case Constants.OI.DRIVE_DPAD_UPRIGHT: return Constants.OI.driveDpadDirection.UP;
                //Bottom half of Drive DPAD
                case Constants.OI.DRIVE_DPAD_DNLEFT:
                case Constants.OI.DRIVE_DPAD_DN:
                case Constants.OI.DRIVE_DPAD_DNRIGHT: return Constants.OI.driveDpadDirection.DOWN;
            }
            return Constants.OI.driveDpadDirection.IDLE;
        }

        //OPERATOR CONTROLS
        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the fwd axis 
         */
        public double getClimbAxisValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(oj.getRawAxis(Constants.OI.OPER_AXIS_CLIMB)) : oj.getRawAxis(Constants.OI.OPER_AXIS_CLIMB);
        }
        /**
         * @return integer value of Operator DPAD
         */
        public int getOperPOV() {
            return oj.getPOV(0);
        }

        //HELPERS
        /**
         * @param inVal double precision input value to apply deadband
         * @return double precision -1 to 1 after applying deadband calculation 
         */
        public static final double applyDeadband(double inVal) {
            return ( Math.abs(inVal) < Constants.OI.OI_JOY_DEADBAND ) ? 0.0 : inVal;
        }
    }
}