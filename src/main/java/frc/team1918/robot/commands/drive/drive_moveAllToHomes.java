
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to move all swerve modules to their home positions, as defined by the calibration process.
 */
public class drive_moveAllToHomes extends CommandBase {
  private final DriveSubsystem m_drive;


  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_moveAllToHomes(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.moveAllToHomes();
  }

  @Override
  public boolean isFinished() {
    return m_drive.isAllTurnAtHome();
  }
}