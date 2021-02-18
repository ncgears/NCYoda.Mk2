
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to lock and unlock drive controls to prevent changes to calibration, etc.
 */
public class drive_lockDriveControls extends CommandBase {
  private final DriveSubsystem m_drive;
  private final BooleanSupplier m_lock;


  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_lockDriveControls(DriveSubsystem subsystem, BooleanSupplier lock) {
    m_drive = subsystem;
    m_lock = lock;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.lockDriveControls(m_lock.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}