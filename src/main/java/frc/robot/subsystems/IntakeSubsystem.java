package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase implements Lifecycle {

  private boolean enabled = false;
  private boolean reversed = false;

  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private final double DEFAULT_INTAKE_SPEED = -.7;
  private static final String INTAKE_SPEED_KEY = "Intake Speed";

 private final DoubleSolenoid intakeLift = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 6);
//  private final DoubleSolenoid intakeLift = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2);
  
  public IntakeSubsystem() {
    RaiseIntake();

    SmartDashboard.setDefaultNumber(INTAKE_SPEED_KEY, DEFAULT_INTAKE_SPEED);
  }

  @Override
  public void teleopInit() {
    reversed = false;
  }

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
  }

  public void reverse() {
    this.reversed = true;
  }

  public void forward() {
    this.reversed = false;
  }

  public void DropIntake() {
    intakeLift.set(Value.kForward);
  }

  public void RaiseIntake() {
    intakeLift.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    double intakeSpeed = 0.0;
    if (enabled) {
      intakeSpeed = SmartDashboard.getNumber(INTAKE_SPEED_KEY, DEFAULT_INTAKE_SPEED);
      if(reversed) {
        intakeSpeed = -intakeSpeed;
      }
    }
    intakeMotor.set(intakeSpeed);
  }
}
