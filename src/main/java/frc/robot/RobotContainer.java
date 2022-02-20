package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DetectBallColorCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;


/*
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // private static final PneumaticsModuleType PneumaticHub = null;
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterFeederSubsystem m_feederSubsystem = new ShooterFeederSubsystem();

  private final XboxController gamepad = new XboxController(2);
  private final Joystick leftJoy = new Joystick(0);
  private final Joystick rightJoy = new Joystick(1);

  private final static double minPressure = 110;
  private final static double maxPressure = 120;

  private final PneumaticHub pneuHub = new PneumaticHub();
  private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    m_drivetrainSubsystem.resetOdometry(new Pose2d(5, 4.2, new Rotation2d(0)));

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
        () -> -OIUtil.modifyAxis((rightJoy.getY())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -OIUtil.modifyAxis((rightJoy.getX())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -OIUtil.modifyAxis((leftJoy.getX())) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> !leftJoy.getRawButton(13)));

    // Configure the button bindings
    configureButtonBindings();

    // Zero Out the Gyroscope
    m_drivetrainSubsystem.zeroGyroscope();

    pneuHub.enableCompressorAnalog(minPressure, maxPressure);

    // Debug telemetry
    CommandScheduler.getInstance().schedule(new CommandBase() {
      @Override
      public void execute() {
        SmartDashboard.putNumber("Gyro", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
      }

      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    });

    // Color Sensor Debug
    CommandScheduler.getInstance().schedule(new DetectBallColorCommand(colorSensor));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Back button zeros the gyroscope
    new Button(gamepad::getBackButton)
        // No requirements because we don't need to interrupt anything
        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // Turn to angle button
    // new Button(() -> rightJoy.getRawButton(11))
    // .whenPressed(new TurnToAngleProfiled(45,
    // m_drivetrainSubsystem).withTimeout(20));

    // Drive to Distance button
    // new Button(rightJoy::getTrigger)
    // .whenPressed(new DriveToDistanceProfiled(1, m_drivetrainSubsystem));

    // new Button(rightJoy::getTrigger).whenPressed(m_shooterSubsystem::enable)
    // .whenReleased(m_shooterSubsystem::disable);

    // Track Goal limelight button
    // new Button(() -> rightJoy.getRawButton(14))
    // .whenPressed(new SimpleTrackTargetCommand(m_visionSubsystem,
    // m_drivetrainSubsystem).withTimeout(20));

    // //Drive to distance button
    // new Button(() -> rightJoy.getRawButton(13))
    // .whenPressed(() -> m_drivetrainSubsystem.resetOdometry(new Pose2d(5, 5, new
    // Rotation2d())));

    // Intake Trigger
    new Button(leftJoy::getTrigger).whenPressed(m_intakeSubsystem::enable)
        .whenReleased(m_intakeSubsystem::disable);

    //new Button(() -> rightJoy.getRawButton(5)).whenPressed(m_intakeSubsystem::RaiseIntake)
    //    .whenReleased(m_intakeSubsystem::DropIntake);



    // new Button(()-> leftJoy.getRawButton(11))
    // .whenPressed(() -> solenoid.set(DoubleSolenoid.Value.kReverse));

    // new Button(()-> leftJoy.getRawButton(12))
    // .whenPressed(()-> solenoid.set(DoubleSolenoid.Value.kOff));


    new Button(rightJoy::getTrigger)
        .whenPressed(m_feederSubsystem::pull)
        .whenReleased(m_feederSubsystem::dontPull);

    new Button(()-> rightJoy.getRawButton(8)).toggleWhenPressed(
      new StartEndCommand(m_shooterSubsystem::enable,m_shooterSubsystem::disable ,m_shooterSubsystem)
      );

    // new Button(gamepad::getYButton)
    // .whenPressed(() -> m_climberSubsystem.setClimberState(ClimberState.kExtend))
    // .whenReleased(() ->
    // m_climberSubsystem.setClimberState(ClimberState.kDisabled));

    // new Button(gamepad::getAButton)
    // .whenPressed(() -> m_climberSubsystem.setClimberState(ClimberState.kClimb))
    // .whenReleased(() -> m_climberSubsystem.setClimberState(ClimberState.kHold));

    new Button(()-> rightJoy.getRawButton(5)).whenPressed(m_intakeSubsystem::RaiseIntake)
    .whenReleased(m_intakeSubsystem::DropIntake);

    new Button(()-> rightJoy.getRawButton(6)).whenPressed(m_shooterSubsystem::shortShot);

    new Button(()-> rightJoy.getRawButton(7)).whenPressed(m_shooterSubsystem::longShot);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

}
