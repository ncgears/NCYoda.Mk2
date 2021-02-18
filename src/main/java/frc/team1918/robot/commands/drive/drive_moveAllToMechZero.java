
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to move all swerve modules to their mechanical home positions, as defined by the constants. 
 * This is typically absolute 0, but if the mechanical team has assembled the swerve module incorrectly, the
 * value may be adjusted to compensate for what was assembled mechanically.
 */
public class drive_moveAllToMechZero extends CommandBase {
  private final DriveSubsystem m_drive;


  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_moveAllToMechZero(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.moveAllToMechZero();
  }

  @Override
  public boolean isFinished() {
    return m_drive.isAllTurnAtMechZero();
  }
}