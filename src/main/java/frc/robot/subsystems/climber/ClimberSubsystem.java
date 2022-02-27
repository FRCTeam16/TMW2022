package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  /*
   * Step 1 is the elevator is not extended
   * Step 2 is the solonoid firing forward
   * Step 3 is the elevator going up to fully hang on solonoid bars
   * Step 4 is the elevator extending
   * Step 5 is the solonoid reversing off of the previous bar
   */

  public enum ClimberStep {
    kStep1(1), kStep2(2), kStep3(3), kStep4(4), kStep5(5);

    private final int value;

    private ClimberStep(int value) {
      this.value = value;
    }

    public static ClimberStep fromValue(int value) {
      for (ClimberStep step : ClimberStep.values()) {
        if (step.value == value) {
          return step;
        }
      }
      throw new IllegalArgumentException("No ClimberStep with a value of " + value + " exists");
    }
  }

  /**
   * The current bar the climber system is at
   */
  public enum CurrentBar {
    kMid(1), kHigh(2), kTraverse(3);

    private final int value;

    private CurrentBar(int value) {
      this.value = value;
    }

    public static CurrentBar fromValue(int value) {
      for (CurrentBar bar : CurrentBar.values()) {
        if (bar.value == value) {
          return bar;
        }
      }
      throw new IllegalArgumentException("No CurrentBar with a value of " + value + " exists");
    }

  }

  private final CANSparkMax climberMotor = new CANSparkMax(Constants.LEFTCLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax followerMotor = new CANSparkMax(Constants.RIGHTCLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 5);
  private final DoubleSolenoid climberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 4);

  // Climb state management
  private ClimberStep climberStep = ClimberStep.kStep1;
  private CurrentBar currentBar = CurrentBar.kMid;

  // Current control state of the subsystem
  enum RunState {
    OpenLoop, ClosedLoop
  }
  private RunState runState = RunState.OpenLoop;
  private ClimberClosedLoopManager closedLoopManager = new ClimberClosedLoopManager(climberMotor);


  public enum Positions {
    Retracted(10.0), ReleaseBar(70.0), Extended(90.0);

    private final double value;
    private Positions(double value) {
      this.value = value;
    }
  }

  // Control targets
  private double openLoopValue = 0.0;
  private double positionSetpoint = 0.0;

  public ClimberSubsystem() {
    climberMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();
    climberMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.follow(climberMotor, true);

    climberSolenoid.set(DoubleSolenoid.Value.kOff);
    climberSolenoid2.set(DoubleSolenoid.Value.kOff);

    OpenLoopClimbCommand.ConfigSmartDashboard();

    SmartDashboard.putNumber("Climber/Closed/Position/Retracted", Positions.Retracted.value);
    SmartDashboard.putNumber("Climber/Closed/Position/ReleaseBar", Positions.ReleaseBar.value);
    SmartDashboard.putNumber("Climber/Closed/Position/Extended", Positions.Extended.value);

    // Toggle open/closed loop control
    SmartDashboard.putData("Climber Open Loop", new InstantCommand(() -> runState = RunState.OpenLoop).withName("Climber Open"));
    SmartDashboard.putData("Climber Closd Loop", new InstantCommand(() -> runState = RunState.ClosedLoop).withName("Climber Closed"));
  }


  public void setOpenLoopSpeed(double value) {
    this.openLoopValue = value;
    this.runState = RunState.OpenLoop;
  }

  public void holdClosedLoopPosition() {
    this.setClosedLoopPosition(this.climberMotor.getEncoder().getPosition());
  }

  public void setClosedLoopPosition(Positions position) {
    double target = 0.0;
    switch (position) {
      case Retracted:
        target = SmartDashboard.getNumber("Climber/Closed/Position/Retracted", position.value);
        break;
      case ReleaseBar:
        target = SmartDashboard.getNumber("Climber/Closed/Position/ReleaseBar", position.value);
        break;
      case Extended:
        target = SmartDashboard.getNumber("Climber/Closed/Position/Extended", position.value);
        break;
    }
    setClosedLoopPosition(target);
  }

  public void setClosedLoopPosition(double position) {
    this.openLoopValue = 0.0;
    this.runState = RunState.ClosedLoop;
    this.closedLoopManager.setTarget(position);
  }

  public void setClimberState(ClimberStep state) {
    this.climberStep = state;
  }

  public void moveSolonoidsForward() {
    climberSolenoid.set(Value.kForward);
    climberSolenoid2.set(Value.kForward);
  }

  public void moveSolenoidsBackward() {
    climberSolenoid.set(Value.kReverse);
    climberSolenoid2.set(Value.kReverse);
  }

  public void moveSolenoidsDefault() {
    climberSolenoid.set(Value.kOff);
    climberSolenoid2.set(Value.kOff);
  }

  /**
   * Main method for performing climb steps
   */
  public void nextClimb() {
    int nextValue = this.climberStep.value + 1;
    if (nextValue > ClimberStep.values().length) {
      // we need to roll the climber target bar and roll over this climber step
    } else {
      this.climberStep = ClimberStep.fromValue(nextValue);
    }
  }

  private void nextBar() {
    int nextValue = this.currentBar.value + 1;
    if (nextValue > CurrentBar.values().length) {

    } else {
      // this.currentBar = CurrentBar.fromValue()
    }
  }

  public void displayTelemetry() {
    SmartDashboard.putNumber("Climber/Climber Encoder", climberMotor.getEncoder().getPosition());
    SmartDashboard.putString("Climber/Climber Step", climberStep.name());
    SmartDashboard.putString("Climber/Current Bar", currentBar.name());
    SmartDashboard.putNumber("Climber/Open/Value", openLoopValue);

    SmartDashboard.putNumber("Climber/Output", climberMotor.getAppliedOutput());
    SmartDashboard.putNumber("Climber/CAmps", climberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/FAmps", followerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/CVel", climberMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Climber/FVel", followerMotor.getEncoder().getVelocity());
  }

  public void zeroClimberEncoder() {
    climberMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    displayTelemetry();
    if (runState == RunState.OpenLoop) {
      this.climberMotor.set(openLoopValue);
    } else {
      closedLoopManager.run();
    }
  }

  /**
   * Expose for some command manipulation
   * @return
   * @deprecated
   */
  CANSparkMax getClimberMotor() {
    return climberMotor;
  }
}
