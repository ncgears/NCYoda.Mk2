
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to enable the absolute encoder.
 */
public class drive_enableAbsEncoder extends CommandBase {
  private final DriveSubsystem m_drive;
  private final BooleanSupplier m_enable;


  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_enableAbsEncoder(DriveSubsystem subsystem, BooleanSupplier enable) {
    m_drive = subsystem;
    m_enable = enable;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.setAllTurnEncoderAbsolute(m_enable.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}