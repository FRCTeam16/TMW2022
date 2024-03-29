package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.DetectBallSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.vision.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Global subsystem declaration and tracking to allow easier injection
 * for commands
 */
public class Subsystems {
    private static Subsystems instance;
    public static DrivetrainSubsystem drivetrainSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ClimberSubsystem climberSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static ShooterFeederSubsystem feederSubsystem;
    public static TurretSubsystem turretSubsystem;
    public static DetectBallSubsystem detectBallSubsystem;
    public static LEDSubsystem ledSubsystem;

    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    private Subsystems() {
        drivetrainSubsystem = new DrivetrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        climberSubsystem = new ClimberSubsystem();
        visionSubsystem = new VisionSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        feederSubsystem = new ShooterFeederSubsystem();
        turretSubsystem = new TurretSubsystem();
        detectBallSubsystem = new DetectBallSubsystem();
        ledSubsystem = new LEDSubsystem();

        lifecycleSubsystems.add(intakeSubsystem);
        lifecycleSubsystems.add(climberSubsystem);
        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(shooterSubsystem);
        lifecycleSubsystems.add(feederSubsystem);
        lifecycleSubsystems.add(turretSubsystem);
        lifecycleSubsystems.add(detectBallSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
