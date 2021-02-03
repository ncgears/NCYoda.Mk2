/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
//import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class drive_autoHome extends CommandBase {
  private final DriveSubsystem m_drive;


  /**
   * Creates a new drive_defaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   */
  public drive_autoHome(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.lockDriveControls(true);
  }

  @Override
  public void execute() {
    m_drive.moveAllToHomes();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.lockDriveControls(false);
  }

  @Override
  public boolean isFinished() {
    //TODO: need to insert delay of ~750ms to give time for wheels to home
    return true;
  }
}