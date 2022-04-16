package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;

public class ClimberSubsystem extends SubsystemBase implements Lifecycle{

 
  private final CANSparkMax climberMotor = new CANSparkMax(Constants.RIGHTCLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax followerMotor  = new CANSparkMax(Constants.LEFTCLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 5);
  private final DoubleSolenoid climberSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 4);

  private final SparkMaxLimitSwitch forwardLimitSwitch = climberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
  private final SparkMaxLimitSwitch backwardLimitSwitch = climberMotor.getReverseLimitSwitch(Type.kNormallyOpen);

 
  // Current control state of the subsystem
  enum RunState {
    OpenLoop, ClosedLoop
  }
  private RunState runState = RunState.OpenLoop;
  private ClimberClosedLoopManager closedLoopManager = new ClimberClosedLoopManager(climberMotor);


  public enum Positions {
    Retracted(0.0), ReleaseBar(-23.0), Extended(-118.0), ShortPull(-60);

    private final double value;
    private Positions(double value) {
      this.value = value;
    }
  }

  // Control targets
  private double openLoopValue = 0.0;

  public ClimberSubsystem() {
    climberMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();
    climberMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.follow(climberMotor, true);

    // Increase follower updates
    // climberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    enableLimitSwitches();
    moveSolenoidsBackward();

    OpenLoopClimbCommand.ConfigSmartDashboard();

    SmartDashboard.putNumber("Climber/Closed/Position/Retracted", Positions.Retracted.value);
    SmartDashboard.putNumber("Climber/Closed/Position/ReleaseBar", Positions.ReleaseBar.value);
    SmartDashboard.putNumber("Climber/Closed/Position/Extended", Positions.Extended.value);
    SmartDashboard.putNumber("Climber/Closed/Position/ShortPull", Positions.ShortPull.value);

    // Toggle open/closed loop control
    SmartDashboard.putData("Climber Open Loop", new InstantCommand(() -> runState = RunState.OpenLoop).withName("Climber Open"));
    SmartDashboard.putData("Climber Closd Loop", new InstantCommand(() -> runState = RunState.ClosedLoop).withName("Climber Closed"));

    // Open Loop Speed Defaults
    SmartDashboard.putNumber("Climber/OpenLoop/Extend Speed", -0.35);
    SmartDashboard.putNumber("Climber/OpenLoop/Pull Speed", 0.2);
  }

  public void enableLimitSwitches() {
    System.out.println("[Climber] Enabling Limit Switches");
    this.forwardLimitSwitch.enableLimitSwitch(true);
    this.backwardLimitSwitch.enableLimitSwitch(true);
  }

  public void disableLimitSwitches() {
    System.out.println("[Climber] Disabling Limit Switches");
    this.forwardLimitSwitch.enableLimitSwitch(false);
    this.backwardLimitSwitch.enableLimitSwitch(false);
  }

  public void enableSoftLimits() {
    this.climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    this.climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void disableSoftLimits() {
    this.climberMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    this.climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void setOpenLoopSpeed(double value) {
    // We could use the actual value and clamp it, etc.
    if (value > 0.1) {
      value = SmartDashboard.getNumber("Climber/OpenLoop/Extend Speed", -0.35);
    } else if (value < -0.1) {
      value = SmartDashboard.getNumber("Climber/OpenLoop/Pull Speed", 0.2);
    } else {
      value = 0;
    }
    this.openLoopValue = value;
    this.runState = RunState.OpenLoop;
  }

  public void holdClosedLoopPosition() {
    this.setClosedLoopPosition(this.climberMotor.getEncoder().getPosition());
  }

  public void setClosedLoopPosition(Positions position) {
    Subsystems.turretSubsystem.disableVisionTracking();
    Subsystems.turretSubsystem.centerTurret();

    if (!Subsystems.turretSubsystem.atZero()) {
      System.out.println("!!! TURRET IS NOT CENTERED !!!");
    }

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
      case ShortPull:
        target = SmartDashboard.getNumber("Climber/Closed/Position/ShortPull", position.value);
        break;
    }
    setClosedLoopPosition(target);
  }

  public void setClosedLoopPosition(double position) {
    this.openLoopValue = 0.0;
    this.runState = RunState.ClosedLoop;
    this.closedLoopManager.setTarget(position);
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

  

  public void displayTelemetry() {
    SmartDashboard.putString("Climber/RunState", runState.name());
    SmartDashboard.putNumber("Climber/Climber Encoder", climberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Encoder", climberMotor.getEncoder().getPosition());
    // SmartDashboard.putString("Climber/Climber Step", climberStep.name());
    // SmartDashboard.putString("Climber/Current Bar", currentBar.name());
    SmartDashboard.putNumber("Climber/Open/Value", openLoopValue);

    SmartDashboard.putNumber("Climber/Output", climberMotor.getAppliedOutput());
    SmartDashboard.putNumber("Climber/CAmps", climberMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/FAmps", followerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/CVel", climberMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Climber/FVel", followerMotor.getEncoder().getVelocity());
  }

  public void zeroClimberEncoder() {
    this.runState = RunState.OpenLoop;
    this.openLoopValue = 0.0;
    var response = climberMotor.getEncoder().setPosition(0);
    System.out.println("===> Zero Climber Response: " + response.name());
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
