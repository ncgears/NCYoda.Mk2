package frc.team1918.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

	private static DriveSubsystem instance;
	private static SwerveModule dtFL, dtFR, dtRL, dtRR;
	private static AHRS gyro;

	public static DriveSubsystem getInstance() {
		if (instance == null)
			instance = new DriveSubsystem();
		return instance;
	}

	public DriveSubsystem() {
	    dtFL = new SwerveModule(Constants.DriveTrain.DT_FL_DRIVE_MC_ID,
				Constants.DriveTrain.DT_FL_TURN_MC_ID, 4.20, 0.01, 0, 200); // Front Left
		dtFR = new SwerveModule(Constants.DriveTrain.DT_FR_DRIVE_MC_ID,
				Constants.DriveTrain.DT_FR_TURN_MC_ID, 4.20, 0.01, 0, 200); // Front Right
		dtRL = new SwerveModule(Constants.DriveTrain.DT_RL_DRIVE_MC_ID,
				Constants.DriveTrain.DT_RL_TURN_MC_ID, 4.20, 0.01, 0, 200); // Rear Left
		dtRR = new SwerveModule(Constants.DriveTrain.DT_RR_DRIVE_MC_ID,
                Constants.DriveTrain.DT_RR_TURN_MC_ID, 4.20, 0.01, 0, 200); // Rear Right

		gyro = new AHRS(SPI.Port.kMXP);
	}

	public static AHRS getgyro() {
        return gyro;
	}

	public static void setDrivePower(double flPower, double frPower, double rlPower, double rrPower) {
	    dtFL.setDrivePower(flPower);
		dtFR.setDrivePower(frPower);
		dtRL.setDrivePower(rlPower);
		dtRR.setDrivePower(rrPower);
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

	private static double l = 21, w = 21, r = Math.sqrt((l * l) + (w * w));

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

	public static void resetAllEnc() {
	    dtFL.resetTurnEnc();
		dtFR.resetTurnEnc();
		dtRL.resetTurnEnc();
		dtRR.resetTurnEnc();
	}

	public static void stopDrive() {
	    dtFL.stopDrive();
		dtFR.stopDrive();
		dtRL.stopDrive();
		dtRR.stopDrive();
	}

	private static double angleToLoc(double angle) {
		if (angle < 0) {
			return .5d + ((180d - Math.abs(angle)) / 360d);
		} else {
			return angle / 360d;
		}
	}

	private static boolean offSetSet = false;

	public static void setOffSets() {
		if (!offSetSet) {
			double flOff = 0, frOff = 0, rlOff = 0, rrOff = 0;
		    dtFL.setTurnPower(0);
			dtRL.setTurnPower(0);
			dtFR.setTurnPower(0);
			dtRR.setTurnPower(0);

			flOff = DriveSubsystem.dtFL.getTurnAbsPos();
			frOff = DriveSubsystem.dtFR.getTurnAbsPos();
			rlOff = DriveSubsystem.dtRL.getTurnAbsPos();
			rrOff = DriveSubsystem.dtRR.getTurnAbsPos();

			System.out.println("flOff: " + flOff);
			System.out.println("frOff: " + frOff);
			System.out.println("rlOff: " + rlOff);
			System.out.println("rrOff: " + rrOff);

			resetAllEnc();
		    dtFL.setEncPos((int) (locSub(flOff, Constants.DriveTrain.DT_FL_HOME) * 4095d));
			dtFR.setEncPos((int) (locSub(frOff, Constants.DriveTrain.DT_FR_HOME) * 4095d));
			dtRL.setEncPos((int) (locSub(rlOff, Constants.DriveTrain.DT_RL_HOME) * 4095d));
			dtRR.setEncPos((int) (locSub(rrOff, Constants.DriveTrain.DT_RR_HOME) * 4095d));
			offSetSet = true;
		}
	}

	public static void resetOffSet() {
		offSetSet = false;
	}

	private static double locSub(double v, double c) {
		if (v - c > 0) {
			return v - c;
		} else {
			return (1 - c) + v;
		}
	}

	
	public static double getgyroAngle() {
		return gyro.getAngle();
	}

	public static double getgyroAngleInRad() {
		return gyro.getAngle() * (Math.PI / 180d);
	}

	public static void setDriveBrakeMode(boolean b) {
		//we do this as a safety feature in case we lose comms to a drive controller, its left in coast mode and doesn't drag the wheel
	    dtFL.setBrakeMode(b);
		dtFR.setBrakeMode(b);
		dtRL.setBrakeMode(b);
		dtRR.setBrakeMode(b);
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

		double ws1 = Math.sqrt((b * b) + (c * c));
		double ws2 = Math.sqrt((b * b) + (d * d));
		double ws3 = Math.sqrt((a * a) + (d * d));
		double ws4 = Math.sqrt((a * a) + (c * c));

		double wa1 = Math.atan2(b, c) * 180 / Math.PI;
		double wa2 = Math.atan2(b, d) * 180 / Math.PI;
		double wa3 = Math.atan2(a, d) * 180 / Math.PI;
		double wa4 = Math.atan2(a, c) * 180 / Math.PI;

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
		DriveSubsystem.setDrivePower(ws4, ws2, ws1, ws3); //TODO: Are we supplying in right order
		DriveSubsystem.setLocation(angleToLoc(wa4), angleToLoc(wa2), angleToLoc(wa1), angleToLoc(wa3));
	}

	public void humanDrive(double fwd, double str, double rot) {
		if (Math.abs(rot) < 0.01)
			rot = 0;

			fwd = Helpers.OI.applyDeadband(fwd);
			str = Helpers.OI.applyDeadband(str);
			rot = Helpers.OI.applyDeadband(rot);
		
		if (fwd == 0.0 && str == 0.0 && rot == 0.0) {
			// setOffSets();
			setDriveBrakeMode(true);
			stopDrive();
		} else {
			setDriveBrakeMode(false);
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
}