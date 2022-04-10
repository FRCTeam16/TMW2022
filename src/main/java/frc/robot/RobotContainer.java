package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.auto.AutoManager;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DetectBallColorCommand;
import frc.robot.commands.RunDMSCommand;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.prefs.SaveWheelOffsets;
import frc.robot.commands.prefs.ZeroWheelOffsets;
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
    @SuppressWarnings("unused")
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
    private final static double maxPressure = 118;

    private final PneumaticHub pneuHub = new PneumaticHub();

    public static Alliance alliance = Alliance.Invalid;

    // private final CameraControl cameraControl = new CameraControl();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        RobotContainer.alliance = DriverStation.getAlliance();

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
        CommandScheduler.getInstance().schedule(new DetectBallColorCommand());
    }

    private void configureButtonBindings() {

        // Back button zeros the gyroscope
        new Button(() -> rightJoy.getRawButton(4)).whenPressed(m_drivetrainSubsystem::zeroGyroscope);

        // Track Goal limelight button
        // new Button(() -> rightJoy.getRawButton(14))
        // .whenPressed(new SimpleTrackTargetCommand(m_visionSubsystem,
        // m_drivetrainSubsystem).withTimeout(20));
        
        configureIntakeButtonBindings();
        configureShooterButtonBindings();
        configureClimberButtonBindings();
        configureTurretButtonBindings();
        configureVisionButtonBindings();
        configureWheelOffsetButtonBindings();
        configureDMSButtonBindings();
    }

    private void configureIntakeButtonBindings() {
        new Button(leftJoy::getTrigger)
                .whenPressed(() -> {
                    m_intakeSubsystem.forward();
                    m_intakeSubsystem.enable();
                })
                .whenReleased(m_intakeSubsystem::disable);

        new Button(() -> leftJoy.getRawButton(4))
                .whenPressed(() -> {
                    Subsystems.intakeSubsystem.reverse();
                    Subsystems.intakeSubsystem.enable();
                })
                .whenReleased(() -> {
                    Subsystems.intakeSubsystem.forward();
                    Subsystems.intakeSubsystem.disable();
                });

        new Button(() -> gamepad.getPOV() == 0).whenPressed(m_intakeSubsystem::RaiseIntake);
        new Button(() -> gamepad.getPOV() == 180).whenPressed(m_intakeSubsystem::DropIntake);

    }

    private void configureShooterButtonBindings() {
        new Button(rightJoy::getTrigger)
                .whenPressed(() -> m_feederSubsystem.pull())
                .whenReleased(m_feederSubsystem::dontPull);

        new Button(() -> rightJoy.getPOV() == 90)
                .whenPressed(() -> m_shooterSubsystem.setProfile(ShooterProfile.Short));

        new Button(() -> rightJoy.getPOV() == 0)
                .whenPressed(() -> m_shooterSubsystem.setProfile(ShooterProfile.Long));

        new Button(() -> rightJoy.getPOV() == 270)
                .whenPressed(() -> m_shooterSubsystem.setProfile(ShooterProfile.LowGoal));

        new Button(() -> rightJoy.getPOV() == 180)
                .whenPressed(() -> m_shooterSubsystem.setProfile(ShooterProfile.Dynamic));

        new Button(() -> gamepad.getStartButton())
                .whenPressed(m_shooterSubsystem::enable);

        new Button(() -> gamepad.getBackButton())
                .whenPressed(m_shooterSubsystem::disable);
        new Button(() -> rightJoy.getRawButton(16)).whenPressed(Subsystems.feederSubsystem::disableQueuing);

        SmartDashboard.putData("Shooter Short", new InstantCommand(() -> m_shooterSubsystem.setProfile(ShooterProfile.Short)).withName("Shoot Short"));
        SmartDashboard.putData("Shooter Long", new InstantCommand(() -> m_shooterSubsystem.setProfile(ShooterProfile.Long)).withName("Shoot Long"));
        SmartDashboard.putData("Shooter Off", new InstantCommand(() -> m_shooterSubsystem.setProfile(ShooterProfile.Dynamic)).withName("Shoot Off"));
        SmartDashboard.putData("Shooter Low Goal", new InstantCommand(() -> m_shooterSubsystem.setProfile(ShooterProfile.LowGoal)).withName("Shoot Low Goal"));

    }

    private void configureClimberButtonBindings() {

        SmartDashboard.putData("Zero Climber Encoder", new RunWithDisabledInstantCommand(Subsystems.climberSubsystem::zeroClimberEncoder).withName("Zero Climber Encoder"));

        SmartDashboard.putData("Climber/Cmd/Enable Soft Limits", new InstantCommand(Subsystems.climberSubsystem::enableSoftLimits).withName("Enable Soft Limits"));
        SmartDashboard.putData("Climber/Cmd/Disable Soft Limits", new InstantCommand(Subsystems.climberSubsystem::disableSoftLimits).withName("Disable Soft Limits"));

        // Smart Dashboard Specific
        SmartDashboard.putData("Disable Climber Soft Limits SD", new InstantCommand(Subsystems.climberSubsystem::disableSoftLimits));
        SmartDashboard.putData("Disable Climber Hard Limits SD", new InstantCommand(Subsystems.climberSubsystem::disableLimitSwitches));

        new Button(() -> leftJoy.getRawButton(11)).whenPressed(Subsystems.climberSubsystem::enableLimitSwitches);
        new Button(() -> leftJoy.getRawButton(12)).whenPressed(Subsystems.climberSubsystem::disableLimitSwitches);


        new Button(() -> (Math.abs(gamepad.getRightY()) > 0.10))
                .whileHeld(() -> {
                    Subsystems.climberSubsystem.setOpenLoopSpeed(-gamepad.getRightY());
                })
                .whenReleased(Subsystems.climberSubsystem::holdClosedLoopPosition);
                // .whenReleased(() -> Subsystems.climberSubsystem.setOpenLoopSpeed(0.0));

        new Button(gamepad::getRightBumper).whenPressed(Subsystems.climberSubsystem::moveSolonoidsForward);
        new Button(gamepad::getLeftBumper).whenPressed(Subsystems.climberSubsystem::moveSolenoidsBackward);
        SmartDashboard.putData("Climber/Cmd/SolenoidOff", new InstantCommand(Subsystems.climberSubsystem::moveSolenoidsDefault));
        SmartDashboard.putData("Climber/Cmd/Retract", new InstantCommand(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.Retracted)).withName("Retract"));
        SmartDashboard.putData("Climber/Cmd/ReleaseBar", new InstantCommand(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.ReleaseBar)).withName("ReleaseBar"));
        SmartDashboard.putData("Climber/Cmd/Extend", new InstantCommand(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.Extended)).withName("Extend"));
        SmartDashboard.putData("Climber/Cmd/Reach", new InstantCommand(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.ShortPull)).withName("Hang to swing"));

        new Button(gamepad::getAButton)
                .whenPressed(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.Retracted));
        new Button(gamepad::getYButton)
                .whenPressed(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.Extended));
        new Button(gamepad::getBButton)
                .whenPressed(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.ReleaseBar));
        new Button(gamepad::getXButton)
                .whenPressed(() -> Subsystems.climberSubsystem.setClosedLoopPosition(Positions.ShortPull));

    }

    private void configureTurretButtonBindings() {
        new Button(() -> (gamepad.getLeftTriggerAxis() > 0.25))
            .whileHeld(Subsystems.turretSubsystem::openBackwards)
            .whenReleased(Subsystems.turretSubsystem::openStop);

        new Button(() -> (gamepad.getRightTriggerAxis() > 0.25))
            .whileHeld(Subsystems.turretSubsystem::openForward)
            .whenReleased(Subsystems.turretSubsystem::openStop);

        SmartDashboard.putData("Turret/EnableVision", new InstantCommand(() -> {
            Subsystems.visionSubsystem.enable();
            Subsystems.turretSubsystem.enableVisionTracking();
        }).withName("Enable Vision Tracking"));

        SmartDashboard.putData("Turret/DisableVision", new InstantCommand(() -> {
            Subsystems.visionSubsystem.disable();
            Subsystems.turretSubsystem.openStop();
        }).withName("Enable Vision Tracking"));

        SmartDashboard.putData("Turret/Zero Turret", new RunWithDisabledInstantCommand(Subsystems.turretSubsystem::zeroEncoder)
        .withName("Zero Turret"));
        SmartDashboard.putData("Turret/Center Turret", new InstantCommand(Subsystems.turretSubsystem::centerTurret).withName("Center Turret"));;

        SmartDashboard.putData("Turret/Cmd/Enable Soft Limits", new InstantCommand(Subsystems.turretSubsystem::enableSoftLimits).withName("Enable Limits"));
        SmartDashboard.putData("Turret/Cmd/Disable Soft Limits", new InstantCommand(Subsystems.turretSubsystem::disableSoftLimits).withName("Disable Limits"));

        // Smart Dashboard specific
        SmartDashboard.putData("Disable Turret Soft Limits SD", new InstantCommand(Subsystems.turretSubsystem::disableSoftLimits));
        SmartDashboard.putData("Enable Soft Limits SD", new InstantCommand(Subsystems.turretSubsystem::enableSoftLimits));
        SmartDashboard.putData("Zero Turret SD", new RunWithDisabledInstantCommand(Subsystems.turretSubsystem::zeroEncoder));

    }

    private void configureVisionButtonBindings() {
        new Button(() -> leftJoy.getRawButton(16)).whenPressed(Subsystems.turretSubsystem::enableVisionTracking);
        new Button(() -> leftJoy.getRawButton(15)).whenPressed(Subsystems.turretSubsystem::disableVisionTracking);
    }

    private void configureWheelOffsetButtonBindings() {
        SmartDashboard.putData("Offset/ZeroWheelOffsets", new ZeroWheelOffsets());
        SmartDashboard.putData("Offset/SaveWheelOffsets", new SaveWheelOffsets());
    }

    private void configureDMSButtonBindings() {
        // new Button(() -> leftJoy.getRawButton(14))
        //     .whenPressed(new RunDMSCommand())
        //     .whenReleased(Subsystems.ledSubsystem::stopDMS);
        
        SmartDashboard.putData("DMS/Start", new RunDMSCommand().withName("Run DMS"));
        SmartDashboard.putData("DMS/Stop", new InstantCommand(() -> Subsystems.ledSubsystem.stopDMS()).withName("Stop DMS"));

        SmartDashboard.putData("DMS/Enable All", new InstantCommand(() -> Subsystems.ledSubsystem.startSubsystem()).withName("Enable LED/DMS Subsystem"));
        SmartDashboard.putData("DMS/Disable All", new InstantCommand(() -> Subsystems.ledSubsystem.stopSubsystem()).withName("Disable LED/DMS Subsystem"));
    }

    // private void configureDebugButtonBindings() {
    //     // Turn to angle button
    //     // new Button(() -> rightJoy.getRawButton(11))
    //     // .whenPressed(new TurnToAngleProfiled(45,
    //     // m_drivetrainSubsystem).withTimeout(20));

    //     // Drive to Distance button
    //     // new Button(rightJoy::getTrigger)
    //     // .whenPressed(new DriveToDistanceProfiled(1, m_drivetrainSubsystem));
    // }



    public Command getAutonomousCommand() {
        return m_autoManager.getSelectedCommand();
    }

    public void disabledInit() {
        m_autoManager.initializeAuto();
    }

    public void disabledPeriodic() {
        m_autoManager.showSelectedAuto();
    }

    /**
     * Handle settings initial robot states for teleop
     */
    public void teleopInit() {
        if (m_climberSubsystem != null) {
            m_climberSubsystem.setOpenLoopSpeed(0.0);  // make this init
        }
        Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.teleopInit());
    }

    public void autoInit() {
        Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.autoInit());
    }

}
