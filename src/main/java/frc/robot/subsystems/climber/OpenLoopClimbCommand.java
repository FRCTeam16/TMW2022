package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenLoopClimbCommand extends CommandBase {

  public enum ElevatorAction {
    Pull, Extend, Hold, Disabled
  }

  private static final double DEFAULT_ELEVDOWN_SPEED = 0.2;
  private static final String ELEVDOWN_SPEED_KEY = "Climber/OpenLoop/Pull Speed";

  private static final double DEFAULT_ELEVUP_SPEED = -0.35;
  private static final String ELEVUP_SPEED_KEY = "Climber/OpenLoop/Extend Speed";

  private static final double DEFAULT_ELEVHOLD_SPEED = 0.0; //-0.15
  private static final String ELEVHOLD_SPEED_KEY = "Climber/OpenLoop/Hold Speed";

  private static final double DEFAULT_DISABLED_SPEED = 0;
  private static final String DISABLED_SPEED_KEY = "Climber/OpenLoop/DISABLED Speed";

  private ElevatorAction targetAction = ElevatorAction.Hold;
  private final ClimberSubsystem climberSystem;

  public static void ConfigSmartDashboard() {
    SmartDashboard.setDefaultNumber(ELEVDOWN_SPEED_KEY, DEFAULT_ELEVDOWN_SPEED);
    SmartDashboard.setDefaultNumber(ELEVUP_SPEED_KEY, DEFAULT_ELEVUP_SPEED);
    SmartDashboard.setDefaultNumber(ELEVHOLD_SPEED_KEY, DEFAULT_ELEVHOLD_SPEED);
    SmartDashboard.setDefaultNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);
  }

  /** Creates a new OpenLoopClimbCommand. */
  public OpenLoopClimbCommand(ElevatorAction targetAction, ClimberSubsystem climberSubsystem) {
    ConfigSmartDashboard();

    this.climberSystem = climberSubsystem;
    this.targetAction = targetAction;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pullSpeed = SmartDashboard.getNumber(ELEVDOWN_SPEED_KEY, DEFAULT_ELEVDOWN_SPEED);
    double holdSpeed = SmartDashboard.getNumber(ELEVHOLD_SPEED_KEY, DEFAULT_ELEVHOLD_SPEED);
    double extendSpeed = SmartDashboard.getNumber(ELEVUP_SPEED_KEY, DEFAULT_ELEVUP_SPEED);
    double defaultSpeed = SmartDashboard.getNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);

    double climberOutput = 0.0;
    switch(targetAction) {
      case Pull:
        climberOutput = pullSpeed;
        break;
      case Extend:
        climberOutput = extendSpeed;
        break;
      case Hold:
        climberOutput = holdSpeed;
        break;
      default:
        climberOutput = defaultSpeed;
    }
    SmartDashboard.putString("Climber/OpenLoop/State2", this.targetAction.name());
    SmartDashboard.putNumber("Climber/OpenLoop/Target2", climberOutput);
    SmartDashboard.putNumber("Climber/OpenLoop/Output2", this.climberSystem.getClimberMotor().getAppliedOutput());

    this.climberSystem.getClimberMotor().set(climberOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
