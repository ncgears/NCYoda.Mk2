
package frc.team1918.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class SwerveModule {
    private WPI_TalonSRX turn; //could be CANSparkMax, WPI_TalonSRX, WPI_TalonFX
    private CANSparkMax drive; //could be CANSparkMax, WPI_TalonSRX, WPI_TalonFX
    private final double FULL_ROTATION = 4096d, TURN_P, TURN_I, TURN_D;
    private final int TURN_IZONE;
    public boolean isDrivePowerInverted = false;

//SparkMAX Java API Doc: https://www.revrobotics.com/content/sw/max/sw-docs/java/index.html

 	/**
	 * Lets make a new Swerve Module
	 * @param driveMC_ID First I gotta know what talon we are using for driving
	 * @param turnMC_ID Next I gotta know what talon we are using to turn
	 * @param tP I probably need to know the P constant for the turning PID
	 * @param tI I probably need to know the I constant for the turning PID
	 * @param tD I probably need to know the D constant for the turning PID
	 * @param tIZone I might not need to know the I Zone value for the turning PID
	 */
   
    public SwerveModule(int driveMC_ID, int turnMC_ID, double tP, double tI, double tD, int tIZone){
        drive = new CANSparkMax(driveMC_ID, MotorType.kBrushless);
        turn = new WPI_TalonSRX(turnMC_ID);

        turn.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff
        turn.set(ControlMode.PercentOutput, 0); //Set controller to disabled
        turn.setNeutralMode(NeutralMode.Brake); //Set controller to brake mode
        turn.configSelectedFeedbackSensor(  FeedbackDevice.CTRE_MagEncoder_Relative, // Local Feedback Source
                                            Constants.Global.PID_PRIMARY,				// PID Slot for Source [0, 1]
                                            Constants.Global.kTimeoutMs);				// Configuration Timeout
/*  //During calibration, we use absolute encoder position
        turn.configSelectedFeedbackSensor(	FeedbackDevice.CTRE_MagEncoder_Absolute, // Local Feedback Source
                                            Constants.PID_PRIMARY,				// PID Slot for Source [0, 1]
                                            Constants.kTimeoutMs);				// Configuration Timeout
*/
        isDrivePowerInverted = false;
        TURN_P = tP;
        TURN_I = tI;
        TURN_D = tD;
        TURN_IZONE = tIZone;
        turn.setInverted(false); //do not invert the motor controller
        //turn.setPID(TURN_P, TURN_I, TURN_D);
        //turn.setIZone(TURN_IZONE);
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
     * @return encoder relative position
     */
    public int getTurnRelPos(){
        //relative position
        return turn.getSensorCollection().getQuadraturePosition();
        //TODO: Resume migration to WPI_TalonSRX
        //https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/Legacy/Migration%20Guide.md
    }

    /**
     * @return encoder absolute position
     */
    public double getTurnAbsPos(){
        //absolute position
        return (turn.getSensorCollection().getPulseWidthPosition() & 0xFFF) / 4095d;
    }

    /**
     * Reset encoder to 0
     */
    public void resetTurnEnc() {
		this.turn.getSensorCollection().setQuadraturePosition(0,10);
    }
    
    public void setEncPos(int d) { //reset encoder position
        turn.getSensorCollection().setQuadraturePosition(d,10);
    }

    /**
     * @return true if the encoder is connected and valid
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

    public int getTurnRotations() {
        return (int) (turn.getSensorCollection().getQuadraturePosition() / FULL_ROTATION);
    }

    public double getTurnLocation() {
        return (turn.getSensorCollection().getQuadraturePosition() % FULL_ROTATION) / FULL_ROTATION;
    }

    public void setTurnPIDToSetPoint(double setpoint) {
        turn.set(ControlMode.Position, setpoint);
    }

    /**
	 * Set turn to pos from 0 to 1 using PID using shortest path to location
	 * @param wa location to set to
	 */	
	public void setTurnLocation(double wa) {
        int currentAngle = getTurnRelPos();
        wa += Math.PI;
        this.isDrivePowerInverted = false;
        int targetAngle = (int) (wa / Math.PI * FULL_ROTATION * 0.5);
        int currentNumRotations = (int) (currentAngle / FULL_ROTATION);
        targetAngle += (currentNumRotations < 0) ? currentNumRotations * FULL_ROTATION : (currentNumRotations + 1) * FULL_ROTATION;

        if ((targetAngle > currentAngle + FULL_ROTATION * 0.25) || (targetAngle < currentAngle - FULL_ROTATION * 0.25)) {
            if (currentAngle < targetAngle) {
                if (targetAngle - currentAngle > FULL_ROTATION * 0.75) {
                    targetAngle -= FULL_ROTATION;
                } else {
                    targetAngle -= FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            } else {
                if ( currentAngle - targetAngle > FULL_ROTATION * 0.75) {
                    targetAngle += FULL_ROTATION;
                } else {
                    targetAngle += FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            }
        }
        turn.set(ControlMode.Position,targetAngle);
    }
    
    public double getError() {
        return turn.getClosedLoopError(); //SUS: To report talon PID loop errors back to robot
    }

    public void stopBoth() {
        setDrivePower(0);
        setTurnPower(0);
    }

    public void stopDrive() {
        setDrivePower(0);
    }

    public void setBrakeMode(boolean b) {
        if(b) { 
            drive.setIdleMode(CANSparkMax.IdleMode.kBrake); //SparkMAX
            //drive.setNeutralMode(NeutralMode.Brake); //TalonSRX
        } else {
            drive.setIdleMode(CANSparkMax.IdleMode.kCoast); //SparkMAX
            //drive.setNeutralMode(NeutralMode.Coast); //TalonSRX
        }
    }
}