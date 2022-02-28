package frc.robot;

import com.revrobotics.ColorSensorV3;

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
import frc.robot.auto.AutoManager;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DetectBallColorCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.Positions;

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

    // Must be called before any subsystem access
    private final Subsystems subsystems = Subsystems.getInstance();

    private final DrivetrainSubsystem m_drivetrainSubsystem = Subsystems.drivetrainSubsystem;
    private final IntakeSubsystem m_intakeSubsystem = Subsystems.intakeSubsystem;
    private final ShooterFeederSubsystem m_feederSubsystem = Subsystems.feederSubsystem;
    private final ShooterSubsystem m_shooterSubsystem = Subsystems.shooterSubsystem;
    private final ClimberSubsystem m_climberSubsystem = Subsystems.climberSubsystem;

    public final AutoManager m_autoManager = new AutoManager();

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
        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
                () -> -OIUtil.modifyAxis((rightJoy.getY())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -OIUtil.modifyAxis((rightJoy.getX())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -OIUtil.modifyAxis((leftJoy.getX()))
                        * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
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
                SmartDashboard.putNumber("Gyro", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() % 360);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });

        // Color Sensor Debug
        CommandScheduler.getInstance().schedule(new DetectBallColorCommand(colorSensor));
    }

    private void configureButtonBindings() {

        // Back button zeros the gyroscope
        new Button(gamepad::getBackButton)
                // No requirements because we don't need to interrupt anything
                .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

        // Track Goal limelight button
        // new Button(() -> rightJoy.getRawButton(14))
        // .whenPressed(new SimpleTrackTargetCommand(m_visionSubsystem,
        // m_drivetrainSubsystem).withTimeout(20));
        
        configureIntakeButtonBindings();
        configureShooterButtonBindings();
        configureClimberButtonBindings();
        configureTurretButtonBindings();
        configureVisionButtonBindings();
    }

    private void configureIntakeButtonBindings() {
        new Button(leftJoy::getTrigger).whenPressed(m_intakeSubsystem::enable)
                .whenReleased(m_intakeSubsystem::disable);

        new Button(() -> rightJoy.getRawButton(5)).whenPressed(m_intakeSubsystem::RaiseIntake)
                .whenReleased(m_intakeSubsystem::DropIntake);
    }

    private void configureShooterButtonBindings() {
        new Button(rightJoy::getTrigger)
                .whenPressed(m_feederSubsystem::pull)
                .whenReleased(m_feederSubsystem::dontPull);

        new Button(() -> rightJoy.getRawButton(8)).toggleWhenPressed(
                new StartEndCommand(m_shooterSubsystem::enable, m_shooterSubsystem::disable, m_shooterSubsystem));

        new Button(() -> rightJoy.getRawButton(6)).whenPressed(m_shooterSubsystem::shortShot);

        new Button(() -> rightJoy.getRawButton(7)).whenPressed(m_shooterSubsystem::longShot);

        SmartDashboard.putData("Shooter Short", new InstantCommand(() -> m_shooterSubsystem.setProfile(ShooterProfile.Short)).withName("Shoot Short"));
        SmartDashboard.putData("Shooter Long", new InstantCommand(() -> m_shooterSubsystem.setProfile(ShooterProfile.Short)).withName("Shoot Long"));

    }

    private void configureClimberButtonBindings() {

        SmartDashboard.putData("Zero Climber Encoder", new InstantCommand(Subsystems.climberSubsystem::zeroClimberEncoder).withName("Zero Climber Encoder"));

        new Button(gamepad::getAButton)
                .whenPressed(() -> {
                    double value = SmartDashboard.getNumber("Climber/OpenLoop/Extend Speed", -0.35); // -0.2
                    m_climberSubsystem.setOpenLoopSpeed(value);
                })
                .whenReleased(() -> {
                    m_climberSubsystem.setOpenLoopSpeed(0.0);
                });

        new Button(gamepad::getYButton)
                .whenPressed(() -> {
                    double value = SmartDashboard.getNumber("Climber/OpenLoop/Pull Speed", 0.2);
                    m_climberSubsystem.setOpenLoopSpeed(value);
                })
                .whenReleased(() -> {
                    m_climberSubsystem.setOpenLoopSpeed(0.0);
                });

        new Button(gamepad::getRightBumper).whenPressed(Subsystems.climberSubsystem::moveSolonoidsForward);
        new Button(gamepad::getLeftBumper).whenPressed(Subsystems.climberSubsystem::moveSolenoidsBackward);
        new Button(gamepad::getStartButton).whenPressed(Subsystems.climberSubsystem::moveSolenoidsDefault);

        SmartDashboard.putData("Climber/Cmd/Retract", new InstantCommand(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.Retracted)).withName("Retract"));
        SmartDashboard.putData("Climber/Cmd/ReleaseBar", new InstantCommand(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.ReleaseBar)).withName("ReleaseBar"));
        SmartDashboard.putData("Climber/Cmd/Extend", new InstantCommand(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.Extended)).withName("Extend"));
    }

    private void configureTurretButtonBindings() {
        new Button(() -> (gamepad.getLeftTriggerAxis() > 0.25))
            .whenPressed(Subsystems.turretSubsystem::openBackwards)
            .whenReleased(Subsystems.turretSubsystem::openStop);

        new Button(() -> (gamepad.getRightTriggerAxis() > 0.25))
            .whenPressed(Subsystems.turretSubsystem::openForward)
            .whenReleased(Subsystems.turretSubsystem::openStop);

        SmartDashboard.putData("Turret/EnableVision", new InstantCommand(() -> {
            Subsystems.visionSubsystem.enable();
            Subsystems.turretSubsystem.enableVisionTracking();
        }).withName("Enable Vision Tracking"));

        SmartDashboard.putData("Turret/DisableVision", new InstantCommand(() -> {
            Subsystems.visionSubsystem.disable();
            Subsystems.turretSubsystem.openStop();
        }).withName("Enable Vision Tracking"));
    }

    private void configureVisionButtonBindings() {
    }

    private void configureDebugButtonBindings() {
        // Turn to angle button
        // new Button(() -> rightJoy.getRawButton(11))
        // .whenPressed(new TurnToAngleProfiled(45,
        // m_drivetrainSubsystem).withTimeout(20));

        // Drive to Distance button
        // new Button(rightJoy::getTrigger)
        // .whenPressed(new DriveToDistanceProfiled(1, m_drivetrainSubsystem));
    }



    public Command getAutonomousCommand() {
        return m_autoManager.getSelectedCommand();
    }

    /**
     * Handle settings initial robot states for teleop
     */
    public void teleopInit() {
        if (m_climberSubsystem != null) {
            m_climberSubsystem.setOpenLoopSpeed(0.0);
        }
    }

}
