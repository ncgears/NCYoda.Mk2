
package frc.team1918.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class SwerveModule {
    private WPI_TalonSRX turn; //could be CANSparkMax, WPI_TalonSRX, WPI_TalonFX
    private CANSparkMax drive; //could be CANSparkMax, WPI_TalonSRX, WPI_TalonFX
    private final double FULL_ROTATION = 4096d, TURN_P, TURN_I, TURN_D;
    private final int TURN_IZONE;
    private boolean isDrivePowerInverted = false;
    private String moduleName;
    private boolean absEncoderEnabled = false;

//SparkMAX Java API Doc: https://www.revrobotics.com/content/sw/max/sw-docs/java/index.html

 	/**
	 * 1918 Swerve Module - Uses Spark Max for drive (Neo) and Talon SRX for turn (bag with gearbox)
	 * @param driveMC_ID This is the CAN ID of the drive motor controller
	 * @param turnMC_ID This is the CAN ID of the turn motor controller
	 * @param tP The P constant (double) for the turning PID
	 * @param tI The I constant (double) for the turning PID
	 * @param tD The D constant (double) for the turning PID
	 * @param tIZone The IZone value (int) for the turning PID
	 */
    public SwerveModule(int driveMC_ID, int turnMC_ID, double tP, double tI, double tD, int tIZone, String name){
        drive = new CANSparkMax(driveMC_ID, MotorType.kBrushless);
        turn = new WPI_TalonSRX(turnMC_ID);
        moduleName = name;

        turn.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff
        turn.set(ControlMode.PercentOutput, 0); //Set controller to disabled
        turn.setNeutralMode(NeutralMode.Brake); //Set controller to brake mode
        turn.configSelectedFeedbackSensor(  FeedbackDevice.CTRE_MagEncoder_Relative, // Local Feedback Source
                                            Constants.Global.PID_PRIMARY,				// PID Slot for Source [0, 1]
                                            Constants.Global.kTimeoutMs);				// Configuration Timeout
        isDrivePowerInverted = false;
        TURN_P = tP;
        TURN_I = tI;
        TURN_D = tD;
        TURN_IZONE = tIZone;
        turn.setInverted(true); //invert turn direction if targetAngle is opposite currentAngle in setTurnLocation
        turn.config_kP(0, tP);
        turn.config_kI(0, tI);
        turn.config_kD(0, tD);
        turn.config_IntegralZone(0, tIZone);
        //turn.setPID(TURN_P, TURN_I, TURN_D);
        //turn.setIZone(TURN_IZONE);
        turn.setSensorPhase(true);
    }

    /**
	 * @param p turn power from -1 to 1
    */
    public void setTurnPower(double p){
        this.turn.set(ControlMode.PercentOutput, p);
    }

    /**
	 * @param p drive motor power from -1 to 1
    */
    public void setDrivePower(double p){
        if (this.isDrivePowerInverted) {
            this.drive.set(-p);
        } else {
            this.drive.set(p);
        }
    }

    /**
     * Gets the position of the relative encoder in encoder ticks
     * @return Integer of relative encoder ticks
     */
    public int getTurnRelPos(){
        return turn.getSensorCollection().getQuadraturePosition();
        //https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/Legacy/Migration%20Guide.md
    }

    /** 
     * Gets the position of the absolute encoder in encoder ticks
     * @return Integer of absolute encoder ticks
     */
    public int getTurnAbsPos(){
        return (turn.getSensorCollection().getPulseWidthPosition() & 0xFFF);
    }

    /**
     * Returns a boolean indicating if the module is at home position within the margin of error defined in constants by DriveTrain.DT_HOME_MARGIN_OF_ERROR
     * @param homePos Absolute encoder value of the target home position.
     * @return Boolean value indicating if this swerve module is at the home position.
     */
    public boolean isTurnAtHome(int homePos) {
        int currentPos = getTurnAbsPos();
        int marginErr = Constants.DriveTrain.DT_HOME_MARGIN_OF_ERROR;
        int offset = (homePos < marginErr || homePos > 4095 - marginErr) ? 1024 : 0;

        int lowHome = homePos + offset - marginErr; //could use this value % 4096
        int highHome = homePos + offset + marginErr;
        
        lowHome -= (lowHome > 4095) ? 4096 : 0;
        highHome -= (highHome > 4095) ? 4096 : 0;
        currentPos -= (currentPos + offset > 4095) ? 4096 : 0;

        if (currentPos + offset <= highHome && currentPos + offset >= lowHome) {
            System.out.println(moduleName + " isTurnAtHome=true; current="+currentPos+"; target="+homePos+";");
            return true;
        } else {
            System.out.println(moduleName + " isTurnAtHome=false; current="+currentPos+"; target="+homePos+";");
            return false;
        }
    }

    /**
     * Resets the relative encoder to 0.
     */
    public void resetTurnEnc() {
        System.out.println(moduleName + " resetTurnEnc");
		turn.getSensorCollection().setQuadraturePosition(0,10);
    }

    /**
     * Sets the relative encoder to a specific value
     * @param value Integer from 0 to 4095 indicating the relative encoder position to set
     */
    public void setEncPos(int value) {
        turn.getSensorCollection().setQuadraturePosition(value,10);
    }

    /**
     * Checks if the turn encoder is connected and valid
     * @return true if the encoder is connected, false otherwise
     */
    public boolean isTurnEncConnected() {
        /**The isSensorPresent() routine had only supported pulse width sensors as these allow for simple 
         * detection of the sensor signal. The getPulseWidthRiseToRiseUs() routine can be used to accomplish 
         * the same task. The getPulseWidthRiseToRiseUs() routine returns zero if the pulse width signal is 
         * no longer present (120ms timeout).
         */
        return (turn.getSensorCollection().getPulseWidthRiseToRiseUs() > 0) ? true : false;
        //isSensorPresent(FeedbackDevice.CTRE_MagEncoder_Relative) == FeedbackDeviceStatus.FeedbackDeviceStatusPresent;
    }

    /**
     * Gets the number of rotations that the relative encoder has detected
     * @return Integer indicating the number of rotations of the relative encoder
     */
    public int getTurnRotations() {
        return (int) (turn.getSensorCollection().getQuadraturePosition() / FULL_ROTATION);
    }

    /**
     * Gets the relative encoder position within the current rotation
     * @return Integer indicating the current location within the current rotation
     */
    public double getTurnLocation() {
        return (turn.getSensorCollection().getQuadraturePosition() % FULL_ROTATION) / FULL_ROTATION;
    }

    /**
	 * Set turn to pos from 0 to 1 using PID using shortest turn to get the wheels aimed the right way
	 * @param wa wheel angle location to set to in radians
	 */	
	public void setTurnLocation(double wa) {
        int currentAngle = getTurnRelPos();
        this.isDrivePowerInverted = false; //Should we store this in a local variable and set it at the end to prevent changing this while in operation?
        int targetAngle = (int) (wa / Math.PI * (FULL_ROTATION * 0.5)); //full rotation * .5 to give us half encoder counts rotation to match with pi
        int currentNumRotations = (int) (currentAngle / FULL_ROTATION);
        targetAngle += (currentNumRotations >= 0) ? currentNumRotations * FULL_ROTATION : (currentNumRotations + 1) * FULL_ROTATION;
        
        if ((targetAngle > currentAngle + FULL_ROTATION * 0.25) || (targetAngle < currentAngle - FULL_ROTATION * 0.25)) { //if target is more than 25% of a rotation either way
            if (currentAngle < targetAngle) { //left strafe
                if (targetAngle - currentAngle > FULL_ROTATION * 0.75) { //if target would require moving less than 75% of a rotation, just go there
                    targetAngle -= FULL_ROTATION;
                } else { //otherwise, turn half a rotation from the target and reverse the drive power
                    targetAngle -= FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            } else { //right strafe
                if ( currentAngle - targetAngle > FULL_ROTATION * 0.75) { //if target would require moving less than 75% of a rotation, just go there
                    targetAngle += FULL_ROTATION;
                } else { //otherwise, turn half a rotation from the target and reverse the drive power
                    targetAngle += FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            }
        }
        turn.set(ControlMode.Position,targetAngle);
        // System.out.println(moduleName + " setTurnLocation="+targetAngle+"; isDrivePowerInverted="+this.isDrivePowerInverted);
    }
    
    /**
     * Gets the closed-loop error. The units depend on which control mode is in use. If closed-loop is seeking a target sensor position, closed-loop error is the difference between target and current sensor value (in sensor units. Example 4096 units per rotation for CTRE Mag Encoder). If closed-loop is seeking a target sensor velocity, closed-loop error is the difference between target and current sensor value (in sensor units per 100ms). If using motion profiling or Motion Magic, closed loop error is calculated against the current target, and not the "final" target at the end of the profile/movement. See Phoenix-Documentation information on units.
     * @return Double precision units of error
     */
    public double getError() {
        return turn.getClosedLoopError();
    }

    /**
     * Stops both turning and driving by setting their respective motor power to 0.
     */
    public void stopBoth() {
        setDrivePower(0);
        setTurnPower(0);
    }

    /**
     * Stops the drive by setting the motor power to 0.
     */
    public void stopDrive() {
        setDrivePower(0);
    }

    /**
     * Sets the brake mode for the motor controller
     * @param device String of either "turn" or "drive" indicating which device to set
     * @param brake Boolean indicating if the brake mode should be set to brake (true) or coast (false)
     */
    public void setBrakeMode(String device, boolean brake) {
        switch (device) {
            case "turn": //turn is a TalonSRX
                if (brake) {
                    turn.setNeutralMode(NeutralMode.Brake);
                } else {
                    turn.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case "drive": //drive is a SparkMAX
                if (brake) {
                    drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
                } else {
                    drive.setIdleMode(CANSparkMax.IdleMode.kCoast);
                }
                break;
        }
    }

    /**
     * Sets the turn power to a specific PercentOutput
     * @param p Double from -1 to 1 indicating the turn power, where 0.0 is stopped
     */
    public void setTurnPowerPercent(double p) {
           turn.set(ControlMode.PercentOutput, p);
    }

    /**
     * Switches the turn encoder to either Absolute or Relative.
     * @param useAbsolute Boolean indicating whether to enable the absolute encoder (true) or the relative encoder (false)
     */
    public void setTurnEncoderAbsolute(boolean useAbsolute) {
        if (useAbsolute) {
            turn.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.Global.PID_PRIMARY, Constants.Global.kTimeoutMs);
        } else {
            turn.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.Global.PID_PRIMARY, Constants.Global.kTimeoutMs);
            if (this.absEncoderEnabled != useAbsolute) {
                //if we just switched to relative, change the setpoint to 0
                setTurnLocationInEncoderTicks(0.0);
            }
        }
        this.absEncoderEnabled = useAbsolute;
    }

    /**
     * Sets the turn position to a specific setpoint using the current encoder (absolute or relative)
     * @param et Encoder Ticks to turn module to.  This depends on which encoder is active.
     */
    public void setTurnLocationInEncoderTicks(double et) {
        // System.out.print(moduleName + " setTurnLocationInEncoderTicks = "+et+"\n");
        turn.set(ControlMode.Position, et);
    }
}