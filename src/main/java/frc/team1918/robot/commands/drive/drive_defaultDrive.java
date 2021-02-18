
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command that runs the drive actions. This passes the OI inputs on to the appropriate drive system (fieldCentricDrive or humanDrive).
 * fieldCentricDrive is simply a call to humanDrive after gyro corrections are made.
 */
public class drive_defaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_strafe;
  private final DoubleSupplier m_rotation;

  /**
   * Creates a new drive_defaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param strafe The control input for driving sideways
   * @param rotation The control input for turning
   */
  public drive_defaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
    m_drive = subsystem;
    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    if (!m_drive.isDriveControlsLocked()){
      if (Constants.DriveTrain.DT_USE_FIELD_CENTRIC) {
        m_drive.fieldCentricDrive(m_forward.getAsDouble(), m_strafe.getAsDouble(), m_rotation.getAsDouble());
      } else {
        m_drive.humanDrive(m_forward.getAsDouble(), m_strafe.getAsDouble(), m_rotation.getAsDouble());
      }
    }
  }
}