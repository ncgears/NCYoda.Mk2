/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.DoubleSupplier;
// import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.Constants;
import frc.team1918.robot.commands.drive_enableAbsEncoder;
import frc.team1918.robot.commands.drive_lockDriveControls;
import frc.team1918.robot.commands.drive_moveAllToHomes;
import frc.team1918.robot.commands.drive_resetAllEnc;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class cg_drive_autoHome extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;


  /**
   * Creates a new command group.
   *
   * @param subsystem The drive subsystem this command will run on.
   */
  public cg_drive_autoHome(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);

    /**
     * Creates a sequential command group with the objects to run in sequence.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new drive_lockDriveControls(m_drive, true),
        new drive_enableAbsEncoder(m_drive, true),
        new drive_moveAllToHomes(m_drive),
        //new WaitCommand(Constants.DriveTrain.DT_HOME_DELAY),  //we dont need this, we added logic to the isFinished of moveAllToHomes
        new drive_resetAllEnc(m_drive),
        new drive_enableAbsEncoder(m_drive, false),
        new drive_lockDriveControls(m_drive, false)
    );
  }
}