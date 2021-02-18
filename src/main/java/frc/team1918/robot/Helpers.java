//OI = Operator Interface
package frc.team1918.robot;

import frc.team1918.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class Helpers {
    public static final class General {
        /**
        * This function takes a value in inches and returns in meters
        * @param inches double precision value in inches
        * @return value in meters
        */
       public final static double inToMeters(double inches) {
           return inches * 0.0254;
       }
    }
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
        // /**
        //  * @return integer value of Drive DPAD
        //  */
        // public int getPOVDrive() {
        //     return dj.getPOV(0);
        // }
        // /**
        //  * This function determines if we are pressing any of the top half of the driver DPAD.
        //  * @return boolean indicating top half of Driver DPAD is pressed
        //  */
        // public boolean isDriveDpadUp() {
        //     switch (getPOVDrive()) {
        //         case Constants.OI.DRIVE_DPAD_UPLEFT:
        //         case Constants.OI.DRIVE_DPAD_UP:
        //         case Constants.OI.DRIVE_DPAD_UPRIGHT: return true;
        //     }
        //     return false;
        // }
        // /**
        //  * This function determines if we are pressing any of the bottom half of the driver DPAD.
        //  * @return boolean indicating bottom half of Drive DPAD is pressed
        //  */
        // public boolean isDriveDpadDown() {
        //     switch (getPOVDrive()) {
        //         case Constants.OI.DRIVE_DPAD_DNLEFT:
        //         case Constants.OI.DRIVE_DPAD_DN:
        //         case Constants.OI.DRIVE_DPAD_DNRIGHT: return true;
        //     }
        //     return false;
        // }
        // /**
        //  * This function returns the direction of the Drive DPAD
        //  * @return enum OI.driveDpadDirection containing one of UP, DOWN, IDLE
        //  */
        // public Constants.OI.driveDpadDirection getDriveDpadDirection() {
        //     switch (getPOVDrive()) {
        //         //Top half of Drive DPAD
        //         case Constants.OI.DRIVE_DPAD_UPLEFT:
        //         case Constants.OI.DRIVE_DPAD_UP:
        //         case Constants.OI.DRIVE_DPAD_UPRIGHT: return Constants.OI.driveDpadDirection.UP;
        //         //Bottom half of Drive DPAD
        //         case Constants.OI.DRIVE_DPAD_DNLEFT:
        //         case Constants.OI.DRIVE_DPAD_DN:
        //         case Constants.OI.DRIVE_DPAD_DNRIGHT: return Constants.OI.driveDpadDirection.DOWN;
        //     }
        //     return Constants.OI.driveDpadDirection.IDLE;
        // }

        //OPERATOR CONTROLS
        /**
         * @param useDeadband Boolean value indicating whether to apply deadband to output
         * @return double precision -1 to 1 of the fwd axis 
         */
        public double getClimbAxisValue(boolean useDeadband) {
            return (useDeadband) ? applyDeadband(oj.getRawAxis(Constants.OI.OPER_AXIS_CLIMB)) : oj.getRawAxis(Constants.OI.OPER_AXIS_CLIMB);
        }
        // /**
        //  * @return integer value of Operator DPAD
        //  */
        // public int getOperPOV() {
        //     return oj.getPOV(0);
        // }

        //HELPERS
        /**
         * @param inVal double precision input value to apply deadband
         * @return double precision -1 to 1 after applying deadband calculation 
         */
        public static final double applyDeadband(double inVal) {
            return ( Math.abs(inVal) < Constants.OI.OI_JOY_DEADBAND ) ? 0.0 : inVal;
        }
        // public static enum DPAD {
        //     UP(0), UP_RIGHT(45), RIGHT(90), DOWN_RIGHT(135), DOWN(180), DOWN_LEFT(225), LEFT(270), UP_LEFT(315);
        //     private int value;
        //     /**
        //      * Constructor
        //      * @param value
        //      */
        //     private DPAD(int value) {
        //         this.value = value;
        //     }

        //     /**
        //      * Convert integers to DPAD values
        //      * @param angle
        //      * @return DPAD with matching angle
        //      */
        //     public static DPAD getEnum(int angle) {
        //         angle = Math.abs(angle);
        //         angle %= 360;
        //         angle = Math.round(angle / 45) * 45; //May have rounding errors, due to rounding errors...
        //         for (DPAD dpad : values()) {
        //             if (dpad.value == angle) return dpad;
        //         }
        //         return null;
        //     }
        // }
        // public static class DirectionalPad extends Button {
        //     private final Joystick parent;
        //     public final Button UP;
        //     public final Button UPRIGHT;
        //     public final Button RIGHT;
        //     public final Button DOWNRIGHT;
        //     public final Button DOWN;
        //     public final Button DOWNLEFT;
        //     public final Button LEFT;
        //     public final Button UPLEFT;

        //     /**
        //      * Constructor
        //      * @param parent - This is the joystick object that owns this dpad
        //      */
        //     DirectionalPad(final Joystick parent) {
        //         this.parent = parent;
        //         this.UP = new DPadButton(this, DPAD.UP);
        //         this.UPRIGHT = new DPadButton(this, DPAD.UP_RIGHT);
        //         this.RIGHT = new DPadButton(this, DPAD.RIGHT);
        //         this.DOWNRIGHT = new DPadButton(this, DPAD.DOWN_RIGHT);
        //         this.DOWN = new DPadButton(this, DPAD.DOWN);
        //         this.DOWNLEFT = new DPadButton(this, DPAD.DOWN_LEFT);
        //         this.LEFT = new DPadButton(this, DPAD.LEFT);
        //         this.UPLEFT = new DPadButton(this, DPAD.UP_LEFT);
        //     }

        //     /**
        //      * This sub-class is used to represent each of the 8 values as a button
        //      */
        //     public static class DPadButton extends Button {
        //         private final DPAD direction;
        //         private final DirectionalPad parent;

        //         /**
        //          * Constructor
        //          * @param parent - This is the DirectionalPad that owns this button
        //          * @param DPAD
        //          */
        //         DPadButton(final DirectionalPad parent, final DPAD dPadDirection) {
        //             this.direction = dPadDirection;
        //             this.parent = parent;
        //         }

        //         @Override
        //         public boolean get() {
        //             return parent.getAngle() == direction.value;
        //         }            
        //     }

        //     private int angle() {
        //         return parent.getPOV();
        //     }

        //     @Override
        //     public boolean get() {
        //         return angle() != -1;
        //     }

        //     /**
        //      * UP           0;
        //      * UP_RIGHT     45;
        //      * RIGHT        90;
        //      * DOWN_RIGHT   135;
        //      * DOWN         180;
        //      * DOWN_LEFT    225;
        //      * LEFT         270;
        //      * UP_LEFT      315;
        //      * @return A number between 0 and 315 indicating dpad direction
        //      */
        //     public int getAngle() {
        //         return angle();
        //     }

        //     /**
        //      * Just like getAngle, but returns a direction instead of an angle
        //      * @return A DPAD direction
        //      */
        //     public DPAD getDirection() {
        //         return DPAD.getEnum(angle());
        //     }
        // }
    }
}