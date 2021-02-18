/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to start the calibration mode.
 * This sets all the turn controllers to coast mode to allow manual adjustment of the swerve modules. 
 * The positions are not saved until we stop calibration mode.
 */
public class drive_startCalibration extends CommandBase {
  private final DriveSubsystem m_drive;


  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_startCalibration(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.startCalibrationMode();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}