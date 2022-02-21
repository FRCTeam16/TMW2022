package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
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

    private Subsystems() {
        drivetrainSubsystem = new DrivetrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        climberSubsystem = new ClimberSubsystem();
        visionSubsystem = new VisionSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        feederSubsystem = new ShooterFeederSubsystem();
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
