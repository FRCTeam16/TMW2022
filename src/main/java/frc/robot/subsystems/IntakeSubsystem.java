package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class IntakeSubsystem extends SubsystemBase {

  private boolean enabled = false;
  private boolean intakeOut = false;

  // private final double DEFAULT_INTAKE_SPEED = -0.5;
  // private static final String INTAKE_SPEED_KEY = "Intake Speed";

  // private final WPI_TalonSRX beaterMotor = new
  // WPI_TalonSRX(Constants.BEATER_MOTOR_ID);
  // private final doub le DEFAULT_BEATERBAR_SPEED = -1;
  // private static final String BEATERBAR_SPEED_KEY = "Beater Bar Speed";

  // Calling CANSpark motor
  private final CANSparkMax intakeMotor;
  private final double DEFAULT_INTAKE_SPEED = -.7;
  private static final String INTAKE_SPEED_KEY = "Intake Speed";

  //private final SolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private final DoubleSolenoid intakeLift = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2,6);
 

  /** Creates a new IntakeSubsystem. */

  public IntakeSubsystem() {
    SmartDashboard.setDefaultNumber(INTAKE_SPEED_KEY, DEFAULT_INTAKE_SPEED);
    // SmartDashboard.setDefaultNumber(BEATERBAR_SPEED_KEY,
    // DEFAULT_BEATERBAR_SPEED);

    this.intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    if (this.intakeMotor.getFirmwareString().equals("0.0.0")) {
      System.out.println("***********************");
    }

  }

  public void enable() {
    this.enabled = true;
    //this.SolenoidPCM.set(true);
  
  }

  public void disable() {
    this.enabled = false;
   // this.SolenoidPCM.set(false);
  }

  public void DropIntake() {
    intakeLift.set(DoubleSolenoid.Value.kForward);
  }

  public void RaiseIntake() {
    intakeLift.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void periodic() {
    double intakeSpeed = 0.0;
    // double beaterSpeed = 0.0;
    if (enabled) {
      intakeSpeed = SmartDashboard.getNumber(INTAKE_SPEED_KEY, DEFAULT_INTAKE_SPEED);
    }
    
  intakeMotor.set(intakeSpeed);
    // set(intakeSpeed);
    // beaterMotor.set(beaterSpeed);
  }
}
