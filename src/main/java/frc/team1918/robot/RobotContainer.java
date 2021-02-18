/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
//Global imports
//import frc.team1918.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
//Util imports
import frc.team1918.robot.utils.AndButton;
import frc.team1918.robot.utils.OrPOVButton;
//Subsystems imports
import frc.team1918.robot.subsystems.ExampleSubsystem;
import frc.team1918.robot.subsystems.ClimberSubsystem;
import frc.team1918.robot.subsystems.CollectorSubsystem;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.ShooterSubsystem;
import frc.team1918.robot.subsystems.MixerSubsystem;
//Commands imports
import frc.team1918.robot.commands.drive.drive_defaultDrive;
import frc.team1918.robot.commands.drive.drive_startCalibration;
import frc.team1918.robot.commands.drive.drive_stopCalibration;
import frc.team1918.robot.commands.drive.drive_resetGyro;
import frc.team1918.robot.commands.drive.drive_moveAllToMechZero;
//samples
import frc.team1918.robot.commands.ExampleCommand;
import frc.team1918.robot.commands.shooter.shooter_shootWall;
//CommandGroup imports
import frc.team1918.robot.commandgroups.cg_drive_autoHome;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Here, we initialize the subsystems and commands that will be used for the button bindings
  //generic subsystems
  private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  //team 1918 subsystems
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final CollectorSubsystem m_collector = new CollectorSubsystem();
  private final MixerSubsystem m_mixer = new MixerSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final DriveSubsystem m_drive = new DriveSubsystem();
  //other subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //team 1918 commands
  private final drive_resetGyro m_resetGyro = new drive_resetGyro(m_drive);
  private final cg_drive_autoHome m_autoHome = new cg_drive_autoHome(m_drive);
  // private final shooter_shootWall m_shooter_shootWall = new shooter_shootWall(shooter);
  // private final shooter_shootShort m_shooter_shootShort = new shooter_shootShort(shooter);
  // private final shooter_shootLine m_shooter_shootLine = new shooter_shootLine(shooter);
  // private final shooter_shootTrench m_shooter_shootTrench = new shooter_shootTrench(shooter);
  //other commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //Here, we are defining the buttons and binding
  //Driver Controller
  private Joystick dj = new Joystick(Constants.OI.OI_JOY_DRIVE);
  private JoystickButton btn_ALLUP = new JoystickButton(dj, Constants.OI.DRIVE_BTN_ALLUP);
  private JoystickButton btn_ANTIGRAV = new JoystickButton(dj, Constants.OI.DRIVE_BTN_ANTIGRAV);
  private JoystickButton btn_MECHZERO_KEY1 = new JoystickButton(dj, Constants.OI.DRIVE_BTN_MECHZERO);
  private JoystickButton btn_HOMESWERVE = new JoystickButton(dj, Constants.OI.DRIVE_BTN_HOMESWERVE);
  private JoystickButton btn_MIXER_FEED = new JoystickButton(dj, Constants.OI.DRIVE_BTN_MIXER_FEED);
  private JoystickButton btn_MIXER_FEEDSTUCK = new JoystickButton(dj, Constants.OI.DRIVE_BTN_MIXER_FEEDSTUCK);
  private JoystickButton btn_CALIBRATE_START = new JoystickButton(dj, Constants.OI.DRIVE_BTN_CALIBRATE_START);
  private JoystickButton btn_CALIBRATE_STOP = new JoystickButton(dj, Constants.OI.DRIVE_BTN_CALIBRATE_STOP);
  private POVButton btn_GYRO_RESET = new POVButton(dj, Constants.OI.DRIVE_DPAD_GYRO_RESET);
  private POVButton btn_THROTUP_UP = new POVButton(dj, Constants.OI.DRIVE_DPAD_THROTUP_UP);
    private POVButton btn_THROTUP_UL = new POVButton(dj, Constants.OI.DRIVE_DPAD_THROTUP_UL);
    private POVButton btn_THROTUP_UR = new POVButton(dj, Constants.OI.DRIVE_DPAD_THROTUP_UR);
  private POVButton btn_THROTDN_DN = new POVButton(dj, Constants.OI.DRIVE_DPAD_THROTDN_DN);
    private POVButton btn_THROTDN_DL = new POVButton(dj, Constants.OI.DRIVE_DPAD_THROTDN_DL);
    private POVButton btn_THROTDN_DR = new POVButton(dj, Constants.OI.DRIVE_DPAD_THROTDN_DR);
  private OrPOVButton orbtn_THROTUP = new OrPOVButton(btn_THROTUP_UP, btn_THROTUP_UL, btn_THROTUP_UR);
  private OrPOVButton orbtn_THROTDN = new OrPOVButton(btn_THROTDN_DN, btn_THROTDN_DL, btn_THROTDN_DR);
  
  //Operator Controller
  private Joystick oj = new Joystick(Constants.OI.OI_JOY_OPER);
  private JoystickButton btn_SHOOT_WALL = new JoystickButton(oj, Constants.OI.OPER_BTN_SHOOT_WALL);
  private JoystickButton btn_SHOOT_LINE = new JoystickButton(oj, Constants.OI.OPER_BTN_SHOOT_LINE);
  private JoystickButton btn_SHOOT_SHORT = new JoystickButton(oj, Constants.OI.OPER_BTN_SHOOT_SHORT);
  private JoystickButton btn_SHOOT_TRENCH = new JoystickButton(oj, Constants.OI.OPER_BTN_SHOOT_TRENCH);
  private JoystickButton btn_TOG_MIDDOWN = new JoystickButton(oj, Constants.OI.OPER_BTN_TOG_MIDDOWN);
  private JoystickButton btn_COLLECTOR_IN = new JoystickButton(oj, Constants.OI.OPER_BTN_COLLECTOR_IN);
  private JoystickButton btn_MECHZERO_KEY2 = new JoystickButton(oj, Constants.OI.OPER_BTN_MECHZERO);

  //Special Bindings
  private AndButton andbtn_MECHZERO = new AndButton(btn_MECHZERO_KEY1,btn_MECHZERO_KEY2);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  
    m_drive.setDefaultCommand(
      new drive_defaultDrive(
          m_drive,
          () -> Helpers.OI.getAxisFwdValue(true),
          () -> Helpers.OI.getAxisStrafeValue(true),
          () -> Helpers.OI.getAxisTurnValue(true)
      )
    );

  
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * For more info, see {@link https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html}
   */
  private void configureButtonBindings() {
    
    btn_HOMESWERVE.whenPressed(new cg_drive_autoHome(m_drive));
    btn_CALIBRATE_START.whenPressed(new drive_startCalibration(m_drive));
    btn_CALIBRATE_STOP.whenPressed(new drive_stopCalibration(m_drive));
    btn_GYRO_RESET.whenPressed(new drive_resetGyro(m_drive));
    //
    btn_SHOOT_WALL.whenPressed(new shooter_shootWall(m_shooter));
    // btn_ALLUP.whenPressed(new moveArmUp(m_collector));
    // btn_ANTIGRAV.whenPressed(new engageAntiBackdrive(m_climber)).whenReleased(new disengageAntiBackdrive(m_climber));

    //bind all 3 up and all 3 down for shooter throttle up/down
    // btn_THROTUP_UP.or(btn_THROTUP_UL).or(btn_THROTUP_UR).whenPressed(new do_something);
    // orbtn_THROTUP.whenPressed(new shooter_increaseThrottle());
    // btn_THROTDN_DN.or(btn_THROTDN_DL).or(btn_THROTDN_DR).whenPressed(new do_something_else);
    // orbtn_THROTDN.whenPressed(new shooter_decreaseThrottle());

    //bind both buttons requirement
    andbtn_MECHZERO.whenPressed(new drive_moveAllToMechZero(m_drive));
    //btn_ABSZERO_KEY1.and(btn_ABSZERO_KEY2).whenPressed(new drive_moveAllToMechZero(m_drive)); //Something something something triggers, couldnt make it work.


  }

  public drive_resetGyro getResetGyroCommand() {
    return m_resetGyro;
  }
  public cg_drive_autoHome getAutoHomeCommand() {
    return m_autoHome;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
