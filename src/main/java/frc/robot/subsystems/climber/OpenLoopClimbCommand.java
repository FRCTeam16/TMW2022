package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenLoopClimbCommand extends CommandBase {

  public enum ElevatorAction {
    Pull, Release, Extend, Hold
  }

  private final double DEFAULT_ELEVDOWN_SPEED = -0.35;
  private static final String ELEVDOWN_SPEED_KEY = "Climber Speed";

  private final double DEFAULT_ELEVUP_SPEED = 0.20;
  private static final String ELEVUP_SPEED_KEY = "Extend Speed";

  private final double DEFAULT_ELEVHOLD_SPEED = -0.15;
  private static final String ELEVHOLD_SPEED_KEY = "Hold Speed";

  private final double DEFAULT_DISABLED_SPEED = 0;
  private static final String DISABLED_SPEED_KEY = "DISABLED Speed";

  private ElevatorAction targetAction = ElevatorAction.Hold;
  private final ClimberSubsystem climberSystem;


  /** Creates a new OpenLoopClimbCommand. */
  public OpenLoopClimbCommand(ElevatorAction targetAction, ClimberSubsystem climberSubsystem) {
    SmartDashboard.setDefaultNumber(ELEVDOWN_SPEED_KEY, DEFAULT_ELEVDOWN_SPEED);
    SmartDashboard.setDefaultNumber(ELEVUP_SPEED_KEY, DEFAULT_ELEVUP_SPEED);
    SmartDashboard.setDefaultNumber(ELEVHOLD_SPEED_KEY, DEFAULT_ELEVHOLD_SPEED);
    SmartDashboard.setDefaultNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);

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
    double releaseSpeed = SmartDashboard.getNumber(ELEVHOLD_SPEED_KEY, DEFAULT_ELEVHOLD_SPEED);
    double extendSpeed = SmartDashboard.getNumber(ELEVUP_SPEED_KEY, DEFAULT_ELEVUP_SPEED);
    double defaultSpeed = SmartDashboard.getNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);

    double climberOutput = 0.0;
    switch(targetAction) {
      case Pull:
        climberOutput = pullSpeed;
        break;
      case Release:
        climberOutput = releaseSpeed;
        break;
      case Extend:
        climberOutput = extendSpeed;
        break;
      case Hold:
        climberOutput = defaultSpeed;
        break;
      default:
        climberOutput = defaultSpeed;
    }

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
