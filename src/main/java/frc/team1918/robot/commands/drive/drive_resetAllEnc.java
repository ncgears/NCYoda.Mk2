
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to reset all the relative encoders to zero. 
 * This happens after we reach a home position to track deviations from the calibrated home position.
 */
public class drive_resetAllEnc extends CommandBase {
  private final DriveSubsystem m_drive;


  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_resetAllEnc(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.resetAllEnc();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}