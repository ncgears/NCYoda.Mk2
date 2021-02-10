
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
     * @return encoder relative position
     */
    public int getTurnRelPos(){
        //relative position
        return turn.getSensorCollection().getQuadraturePosition();
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
        System.out.print(moduleName + " resetTurnEnc\n");
		turn.getSensorCollection().setQuadraturePosition(0,10);
    }
// Why does the line above start with "this." and the one below does not? Does it matter?
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
	 * @param wa location to set to in radians
	 */	
	public void setTurnLocation(double wa) {
        // SmartDashboard.putNumber("wa", wa);
        int currentAngle = getTurnRelPos();
        // SmartDashboard.putNumber("currentAngle", currentAngle);
        //wa += Math.PI;
        this.isDrivePowerInverted = false;
        int targetAngle = (int) (wa / Math.PI * (FULL_ROTATION * 0.5)); //full rotation * .5 to give us half encoder counts rotation to match with pi
        // SmartDashboard.putNumber("targetAngle1", targetAngle);
        int currentNumRotations = (int) (currentAngle / FULL_ROTATION);
        // SmartDashboard.putNumber("numRotation", currentNumRotations);
        //example: current encoder count of 5500, shift target angle into proper rotation set (ie, 0-4096; 4097-8192)
        targetAngle += (currentNumRotations >= 0) ? currentNumRotations * FULL_ROTATION : (currentNumRotations + 1) * FULL_ROTATION;
        // SmartDashboard.putNumber("targetAngle2", targetAngle);
        
        if ((targetAngle > currentAngle + FULL_ROTATION * 0.25) || (targetAngle < currentAngle - FULL_ROTATION * 0.25)) {
            if (currentAngle < targetAngle) { //left strafe
                // SmartDashboard.putString("target","left strafe");
                if (targetAngle - currentAngle > FULL_ROTATION * 0.75) {
                    targetAngle -= FULL_ROTATION;
                } else {
                    targetAngle -= FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            } else { //right strafe ... currentAngle > targetAngle
                // SmartDashboard.putString("target","right strafe");
                if ( currentAngle - targetAngle > FULL_ROTATION * 0.75) { 
                //if ( targetAngle - currentAngle < (FULL_ROTATION * 0.75 * -1)) {
                    // SmartDashboard.putString("target2","if");
                    targetAngle += FULL_ROTATION;
                } else {
                    // SmartDashboard.putString("target2","else");  
                    targetAngle += FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            }
            // SmartDashboard.putString("target","outside quarter rotation");
        // } else {
            // SmartDashboard.putString("target","within quarter rotation");
        }
        // SmartDashboard.putBoolean("drivePowerInverted",this.isDrivePowerInverted);
        // SmartDashboard.putNumber("targetAngleOut", targetAngle);
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

    public void setBrakeMode(String dev, boolean b) {
        switch (dev) {
            case "turn": //turn is a TalonSRX
                if (b) {
                    turn.setNeutralMode(NeutralMode.Brake);
                } else {
                    turn.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case "drive": //drive is a SparkMAX
                if (b) {
                    drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
                } else {
                    drive.setIdleMode(CANSparkMax.IdleMode.kCoast);
                }
                break;
        }
    }

    public void setTurnPowerPercent(double p) {
           turn.set(ControlMode.PercentOutput, p);
    }

    public void setTurnLocationInEncoderTicks(double et) {
        turn.set(ControlMode.Position, et);
    }
}