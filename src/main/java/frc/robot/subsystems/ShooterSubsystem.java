// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// //import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;




// public class ShooterSubsystem extends SubsystemBase {

//   private boolean enabled = false;

//   //private final WPI_TalonSRX intakeMotor = Constants.Objects.INTAKE_MOTOR;
//   //private final double DEFAULT_INTAKE_SPEED = -0.5;
//   //private static final String INTAKE_SPEED_KEY = "Intake Speed";

//   // private final WPI_TalonSRX beaterMotor = new WPI_TalonSRX(Constants.BEATER_MOTOR_ID);
//   //private final double DEFAULT_BEATERBAR_SPEED = -1;
//   //private static final String BEATERBAR_SPEED_KEY = "Beater Bar Speed";

//   // Calling CANSpark motor 

//    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.BEATER_MOTOR_ID, MotorType.kBrushless);
//    private final double DEFAULT_FEEDER_SPEED = -0.5;
//    private static final String FEEDER_SPEED_KEY = "Intake Speed";


//   /** Creates a new ShooterSubsystem. */
 
//   public ShooterSubsystem() {
//     SmartDashboard.setDefaultNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
//     //SmartDashboard.setDefaultNumber(BEATERBAR_SPEED_KEY, DEFAULT_BEATERBAR_SPEED);
//   }

//   public void enable() {
//     this.enabled = true;
//   }

//   public void disable() {
//     this.enabled = false;
//   }

//   @Override
//   public void periodic() {
//     double feederSpeed = 0.0;
   
//     if (enabled) {
//       feederSpeed = SmartDashboard.getNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
//     }
//     intakeMotor.set(feederSpeed);
    

  
//   }


// }

