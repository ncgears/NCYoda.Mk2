package frc.team1918.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//file read/write operations
import java.io.BufferedWriter;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileWriter;
import java.io.FileReader;
import java.io.IOException;
//kinematics and odometry
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;

public class DriveSubsystem extends SubsystemBase {

	private static DriveSubsystem instance;
	private static SwerveModule dtFL, dtFR, dtRL, dtRR;
	private static AHRS gyro;
	private static int flHome = Constants.DriveTrain.DT_FL_MECHZERO;
	private static int frHome = Constants.DriveTrain.DT_FR_MECHZERO;
	private static int rlHome = Constants.DriveTrain.DT_RL_MECHZERO;
	private static int rrHome = Constants.DriveTrain.DT_RR_MECHZERO;
	private static boolean isFirstTime = true;
	private File f;
	private BufferedWriter bw;
	private FileWriter fw;
	private BufferedReader br;
	private FileReader fr;
	private static double l = Constants.Global.ROBOT_LENGTH, w = Constants.Global.ROBOT_WIDTH, r = Math.sqrt((l * l) + (w * w));
	private static boolean driveControlsLocked = false; //true while homing operation

	// private final Translation2d m_FL_LOCATION = new Translation2d(Helpers.General.inToMeters(Constants.Global.ROBOT_WIDTH/2), Helpers.General.inToMeters(Constants.Global.ROBOT_LENGTH/2));
	// private final Translation2d m_FR_LOCATION = new Translation2d(Helpers.General.inToMeters(Constants.Global.ROBOT_WIDTH/2), -Helpers.General.inToMeters(Constants.Global.ROBOT_LENGTH/2));
	// private final Translation2d m_RL_LOCATION = new Translation2d(-Helpers.General.inToMeters(Constants.Global.ROBOT_WIDTH/2), Helpers.General.inToMeters(Constants.Global.ROBOT_LENGTH/2));
	// private final Translation2d m_RR_LOCATION = new Translation2d(-Helpers.General.inToMeters(Constants.Global.ROBOT_WIDTH/2), -Helpers.General.inToMeters(Constants.Global.ROBOT_LENGTH/2));

	public static DriveSubsystem getInstance() {
		if (instance == null)
			instance = new DriveSubsystem();
		return instance;
	}

	public DriveSubsystem() {
	    dtFL = new SwerveModule(Constants.DriveTrain.DT_FL_DRIVE_MC_ID,
				Constants.DriveTrain.DT_FL_TURN_MC_ID, 
				Constants.DriveTrain.DT_TURN_P, 
				Constants.DriveTrain.DT_TURN_I, 
				Constants.DriveTrain.DT_TURN_D, 
				Constants.DriveTrain.DT_TURN_IZONE, "dtFL"); // Front Left
		dtFR = new SwerveModule(Constants.DriveTrain.DT_FR_DRIVE_MC_ID,
				Constants.DriveTrain.DT_FR_TURN_MC_ID, 
				Constants.DriveTrain.DT_TURN_P, 
				Constants.DriveTrain.DT_TURN_I, 
				Constants.DriveTrain.DT_TURN_D, 
				Constants.DriveTrain.DT_TURN_IZONE, "dtFR"); // Front Right
		dtRL = new SwerveModule(Constants.DriveTrain.DT_RL_DRIVE_MC_ID,
				Constants.DriveTrain.DT_RL_TURN_MC_ID, 
				Constants.DriveTrain.DT_TURN_P, 
				Constants.DriveTrain.DT_TURN_I, 
				Constants.DriveTrain.DT_TURN_D, 
				Constants.DriveTrain.DT_TURN_IZONE, "dtRL"); // Rear Left
		dtRR = new SwerveModule(Constants.DriveTrain.DT_RR_DRIVE_MC_ID,
				Constants.DriveTrain.DT_RR_TURN_MC_ID, 
				Constants.DriveTrain.DT_TURN_P,
				Constants.DriveTrain.DT_TURN_I, 
				Constants.DriveTrain.DT_TURN_D, 
				Constants.DriveTrain.DT_TURN_IZONE, "dtRR"); // Rear Right

		gyro = new AHRS(SPI.Port.kMXP);
	}

	public static AHRS getgyro() {
        return gyro;
	}

	public static void setDrivePower(double flPower, double frPower, double rlPower, double rrPower) {
		if (!Constants.DriveTrain.DT_DRIVE_DISABLED) {
			dtFL.setDrivePower(flPower);
			dtFR.setDrivePower(frPower);
			dtRL.setDrivePower(rlPower);
			dtRR.setDrivePower(rrPower);
		}
	}

	public static void setTurnPower(double flPower, double frPower, double rlPower, double rrPower) {
	    dtFL.setTurnPower(flPower);
		dtFR.setTurnPower(frPower);
		dtRL.setTurnPower(rlPower);
		dtRR.setTurnPower(rrPower);
	}

	public static void setLocation(double flLoc, double frLoc, double rlLoc, double rrLoc) {
	    dtFL.setTurnLocation(flLoc);
		dtFR.setTurnLocation(frLoc);
		dtRL.setTurnLocation(rlLoc);
		dtRR.setTurnLocation(rrLoc);
	}

	public static void setAllTurnPower(double power) {
		setTurnPower(power, power, power, power);
	}

	public static void setAllDrivePower(double power) {
		setDrivePower(power, power, power, power);
	}

	public static void setAllLocation(double loc) {
		setLocation(loc, loc, loc, loc);
	}

	public static boolean isdtLFTurnEncConnected() {
		return dtFL.isTurnEncConnected();
	}

	public static boolean isdtFRTurnEncConnected() {
		return dtFR.isTurnEncConnected();
	}

	public static boolean isdtRLTurnEncConnected() {
		return dtRL.isTurnEncConnected();
	}

	public static boolean isBigSushiTurnEncConnected() {
		return dtRR.isTurnEncConnected();
	}

	public void resetAllEnc() {
		System.out.println("resetAllEnc");
	    dtFL.resetTurnEnc();
		dtFR.resetTurnEnc();
		dtRL.resetTurnEnc();
		dtRR.resetTurnEnc();
	}

	public static void stopAllDrive() {
	    dtFL.stopDrive();
		dtFR.stopDrive();
		dtRL.stopDrive();
		dtRR.stopDrive();
	}

	public void resetGyro() {
		System.out.println("resetGyro");
		gyro.reset();
	}

	public static double getgyroAngle() {
		return gyro.getAngle();
	}

	public static double getgyroAngleInRad() {
		return gyro.getAngle() * (Math.PI / 180d);
	}

	public static void setAllDriveBrakeMode(boolean b) {
	    dtFL.setBrakeMode("drive", b);
		dtFR.setBrakeMode("drive", b);
		dtRL.setBrakeMode("drive", b);
		dtRR.setBrakeMode("drive", b);
	}

	public static void setAllTurnBrakeMode(boolean b) {
	    dtFL.setBrakeMode("turn", b);
		dtFR.setBrakeMode("turn", b);
		dtRL.setBrakeMode("turn", b);
		dtRR.setBrakeMode("turn", b);
	}

	public static double getAverageError() {
		return (Math.abs(dtFL.getError()) + Math.abs(dtFR.getError())
				+ Math.abs(dtRL.getError()) + Math.abs(dtRR
				.getError())) / 4d;
	}

	/*
	 * Drive methods
	 */
	public static void swerveDrive(double fwd, double str, double rot) {
		//TODO: If fwd, str, and rot is all 0.0, shortcut this math and do a static thing to save cpu
		double a = str - (rot * (l / r));
		double b = str + (rot * (l / r));
		double c = fwd - (rot * (w / r));
		double d = fwd + (rot * (w / r));
		
		//Wheel Speed
		double ws1 = Math.sqrt((b * b) + (c * c)); //FR
		double ws2 = Math.sqrt((a * a) + (c * c)); //RR
		double ws3 = Math.sqrt((a * a) + (d * d)); //RL
		double ws4 = Math.sqrt((b * b) + (d * d)); //FL

		//Wheel Angle
		double wa1 = Math.atan2(b, c); //FR
		double wa2 = Math.atan2(a, c); //RR
		double wa3 = Math.atan2(a, d); //RL
		double wa4 = Math.atan2(b, d); //FL
		
		double max = ws1;
		max = Math.max(max, ws2);
		max = Math.max(max, ws3);
		max = Math.max(max, ws4);
		if (max > 1) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}
		SmartDashboard.putNumber("ws1", ws1);
		SmartDashboard.putNumber("ws2", ws2);
		SmartDashboard.putNumber("ws3", ws3);
		SmartDashboard.putNumber("ws4", ws4);
		SmartDashboard.putNumber("wa1", wa1);
		SmartDashboard.putNumber("wa2", wa2);
		SmartDashboard.putNumber("wa3", wa3);
		SmartDashboard.putNumber("wa4", wa4);

		DriveSubsystem.setDrivePower(ws4, ws1, ws3, ws2);
		DriveSubsystem.setLocation(wa4, wa1, wa3, wa2);
	}

	public void humanDrive(double fwd, double str, double rot) {
		// System.out.println("humanDrive: fwd="+fwd+"; str="+str+"; rot="+rot);
		if (isFirstTime) {
			//dtFL.setEncPos(0); //same as dtFL.resetTurnEnc()
			resetAllEnc();
			isFirstTime = false;
		}

		SmartDashboard.putNumber("fwd", fwd);
		SmartDashboard.putNumber("str", str);
		SmartDashboard.putNumber("rot", rot);
		
		// if (Math.abs(rot) < 0.01) rot = 0;

		fwd = Helpers.OI.applyDeadband(fwd);
		str = Helpers.OI.applyDeadband(str);
		if (Constants.DriveTrain.DT_TURN_MULT_BEFORE_DB) {
			if (fwd == 0.0 && str == 0.0) {
				rot *= Constants.DriveTrain.DT_TURN_MULT_STATIONARY;
			} else {
				rot *= Constants.DriveTrain.DT_TURN_MULT_MOVING;
			}
		}
		rot = Helpers.OI.applyDeadband(rot);
		if (!Constants.DriveTrain.DT_TURN_MULT_BEFORE_DB) {
			if (fwd == 0.0 && str == 0.0) {
				rot *= Constants.DriveTrain.DT_TURN_MULT_STATIONARY;
			} else {
				rot *= Constants.DriveTrain.DT_TURN_MULT_MOVING;
			}
		}

		if (fwd == 0.0 && str == 0.0 && rot == 0.0) {
			// setOffSets();
			setAllDriveBrakeMode(true);
			stopAllDrive();
		} else {
			setAllDriveBrakeMode(false);
			swerveDrive(fwd, str, rot);
			// resetOffSet();
		}
	}

	public void fieldCentricDrive(double fwd, double str, double rot) {
		double temp = (fwd * Math.cos(getgyroAngleInRad()))
				+ (str * Math.sin(getgyroAngleInRad()));
		str = (-fwd * Math.sin(getgyroAngleInRad()))
				+ (str * Math.cos(getgyroAngleInRad()));
		fwd = temp;
		humanDrive(fwd, str, rot);
	}

	public void tankDrive(double left, double right) {
		setAllLocation(0);
		setDrivePower(right, left, right, left);
	}

	public void getAllAbsPos() {
		flHome = dtFL.getTurnAbsPos();
		frHome = dtFR.getTurnAbsPos();
		rlHome = dtRL.getTurnAbsPos();
		rrHome = dtRR.getTurnAbsPos();

		String outString = "flHome:"+flHome;
		outString += " frHome:"+frHome;
		outString += " rlHome:"+rlHome;
		outString += " rrHome:"+rrHome;
		System.out.println("getAllAbsPos: " + outString);
	}

	public void saveAllHomes() {
		try {
    		f = new File(Constants.DriveTrain.DT_HOMES_FILE);
    		if(!f.exists()){
    			f.createNewFile();
    		}
			fw = new FileWriter(f);
		} catch (IOException e) {
			e.printStackTrace();
		}
		bw = new BufferedWriter(fw);
		String outString = "flHome:"+flHome+"\n";
		outString += "frHome:"+frHome+"\n";
		outString += "rlHome:"+rlHome+"\n";
		outString += "rrHome:"+rrHome+"\n";
		System.out.print("saveAllHomes: " + outString);

		try {
			bw.write(outString);
			bw.close();
			fw.close();
			//TODO: Log to console what data was written to disk
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void readAllHomes() {
		//Read values from file and store in private variables
		f = new File(Constants.DriveTrain.DT_HOMES_FILE);
		if(!f.exists()){
			saveAllHomes();
		}
		try {
			fr = new FileReader(f);
			br = new BufferedReader(fr);
			String line = br.readLine(); //read all the lines from the file and beg for bread
			while (line != null) {
				// System.out.print("readAllHomes line="+line+"\n");
				String part[] = line.split(":",2);
				// System.out.print("readAllHomes part0="+part[0]+"\n");
				switch (part[0]) {
					case "flHome":
						flHome = Integer.parseInt(part[1]);
						// System.out.print("readAllHomes: flHome="+flHome+"\n");
						break;
					case "frHome":
						frHome = Integer.parseInt(part[1]);
						// System.out.print("readAllHomes: frHome="+frHome+"\n");
						break;
					case "rlHome":
						rlHome = Integer.parseInt(part[1]);
						// System.out.print("readAllHomes: rlHome="+rlHome+"\n");
						break;
					case "rrHome":
						rrHome = Integer.parseInt(part[1]);
						// System.out.print("readAllHomes: rrHome="+rrHome+"\n");
						break;
				}
				line = br.readLine(); //beg for more bread
			}
			br.close();
			fr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void moveAllToHomes() {
		System.out.println("moveAllToHomes");
		readAllHomes();
		dtFL.setTurnLocationInEncoderTicks(flHome);
		dtFR.setTurnLocationInEncoderTicks(frHome);
		dtRL.setTurnLocationInEncoderTicks(rlHome);
		dtRR.setTurnLocationInEncoderTicks(rrHome);
	}

	public void moveAllToMechZero() {
		System.out.println("moveAllToMechZero");
		dtFL.setTurnLocationInEncoderTicks(Constants.DriveTrain.DT_FL_MECHZERO);
		dtFR.setTurnLocationInEncoderTicks(Constants.DriveTrain.DT_FR_MECHZERO);
		dtRL.setTurnLocationInEncoderTicks(Constants.DriveTrain.DT_RL_MECHZERO);
		dtRR.setTurnLocationInEncoderTicks(Constants.DriveTrain.DT_RR_MECHZERO);
	}
	public void startCalibrationMode() {
		System.out.println("startCalibrationMode");
		lockDriveControls(true);
		setAllTurnBrakeMode(false);

		dtFL.setTurnPowerPercent(0);
		dtFR.setTurnPowerPercent(0);
        dtRL.setTurnPowerPercent(0);
        dtRR.setTurnPowerPercent(0);
	}

	public void stopCalibrationMode() {
		System.out.println("stopCalibrationMode");
		getAllAbsPos();
		saveAllHomes();
		// readAllHomes();
		setAllTurnBrakeMode(true);
		lockDriveControls(false);
	}

	public boolean isDriveControlsLocked() {
		return driveControlsLocked;
	}

	public void lockDriveControls(boolean lock) {
		driveControlsLocked = lock;
		System.out.println("drive controls lock state: " + lock);
	}

	public void setAllTurnEncoderAbsolute(boolean useAbsolute) {
		dtFL.setTurnEncoderAbsolute(useAbsolute);
		dtFR.setTurnEncoderAbsolute(useAbsolute);
		dtRL.setTurnEncoderAbsolute(useAbsolute);
		dtRR.setTurnEncoderAbsolute(useAbsolute);
	}

	public boolean isAllTurnAtHome() {
		if (
			dtFL.isTurnAtHome(flHome) &&
			dtFR.isTurnAtHome(frHome) &&
			dtRL.isTurnAtHome(rlHome) &&
			dtRR.isTurnAtHome(rrHome)
			) {
				return true;
			}
		return false;
	}

	public boolean isAllTurnAtMechZero() {
		if (
			dtFL.isTurnAtHome(Constants.DriveTrain.DT_FL_MECHZERO) &&
			dtFR.isTurnAtHome(Constants.DriveTrain.DT_FR_MECHZERO) &&
			dtRL.isTurnAtHome(Constants.DriveTrain.DT_RL_MECHZERO) &&
			dtRR.isTurnAtHome(Constants.DriveTrain.DT_RR_MECHZERO)
			) {
				return true;
			}
		return false;
	}
}