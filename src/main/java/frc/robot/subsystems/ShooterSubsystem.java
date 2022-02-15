package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;




public class ShooterSubsystem extends SubsystemBase {

  private boolean enabled = false;

  //Defining Talon, Just incase
  //private final WPI_TalonSRX intakeMotor = Constants.Objects.INTAKE_MOTOR;
  //private final double DEFAULT_INTAKE_SPEED = -0.5;
  //private static final String INTAKE_SPEED_KEY = "Intake Speed";

  // Calling CANSpark motor 

  //  private final CANSparkMax feederMotor = new CANSparkMax(Constants.SHOOTERFEEDER_MOTOR_ID, MotorType.kBrushless);
  //  private final double DEFAULT_FEEDER_SPEED = -.4;
  //  private static final String FEEDER_SPEED_KEY = "Feeder Speed";

   private final CANSparkMax rightShooterMotor = new CANSparkMax(Constants.SHOOTERWHEELRIGHT_MOTOR_ID, MotorType.kBrushless);
   private final double DEFAULT_SHOOTER_SPEED = .3;
   private static final String SHOOTER_SPEED_KEY = "Shooter Speed";
   private final CANSparkMax followerMotor = new CANSparkMax(Constants.SHOOTERWHEELLEFT_MOTOR_ID, MotorType.kBrushless);
   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

   

  /** Creates a new ShooterSubsystem. */
 
  public ShooterSubsystem() {
    followerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //SmartDashboard.setDefaultNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
    SmartDashboard.setDefaultNumber(SHOOTER_SPEED_KEY, DEFAULT_SHOOTER_SPEED);
    followerMotor.follow(rightShooterMotor, true);
    

    // kP = 6e-5; 
    // kI = 0;
    // kD = 0; 
    // kIz = 0; 
    // kFF = 0.000015; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;

    // set PID coefficients
    // var pidController = rightShooterMotor.getPIDController();
    // pidController.setP(kP);
    // pidController.setI(kI);
    // pidController.setD(kD);
    // pidController.setIZone(kIz);
    // pidController.setFF(kFF);
    // pidController.setOutputRange(kMinOutput, kMaxOutput);

    // // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("Shooter/P Gain", kP);
    // SmartDashboard.putNumber("Shooter/I Gain", kI);
    // SmartDashboard.putNumber("Shooter/D Gain", kD);
    // SmartDashboard.putNumber("Shooter/I Zone", kIz);
    // SmartDashboard.putNumber("Shooter/Feed Forward", kFF);
    // SmartDashboard.putNumber("Shooter/Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Shooter/Min Output", kMinOutput);


  }

  public void enable() {
    this.enabled = true;
   
  }

  public void disable() {
    this.enabled = false;
  
  }

  @Override
  public void periodic() {
    //double feederSpeed = 0.0;
    double rightShooterSpeed = 0.0;
   
    if (enabled) {
      //feederSpeed = SmartDashboard.getNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
      rightShooterSpeed = SmartDashboard.getNumber(SHOOTER_SPEED_KEY, DEFAULT_SHOOTER_SPEED);
      
    }

    //feederMotor.set(feederSpeed);
    rightShooterMotor.set(rightShooterSpeed);
    

    // // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("Shooter/P Gain", 0);
    // double i = SmartDashboard.getNumber("Shooter/I Gain", 0);
    // double d = SmartDashboard.getNumber("Shooter/D Gain", 0);
    // double iz = SmartDashboard.getNumber("Shooter/I Zone", 0);
    // double ff = SmartDashboard.getNumber("Shooter/Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Shooter/Max Output", 0);
    // double min = SmartDashboard.getNumber("Shooter/Min Output", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // var pidController = rightShooterMotor.getPIDController();
    // if((p != kP)) { pidController.setP(p); kP = p; }
    // if((i != kI)) { pidController.setI(i); kI = i; }
    // if((d != kD)) { pidController.setD(d); kD = d; }
    // if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   pidController.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 
    // }

    // rightShooterMotor.getPIDController().setReference(200, ControlType.kVelocity);
  
  }


}

