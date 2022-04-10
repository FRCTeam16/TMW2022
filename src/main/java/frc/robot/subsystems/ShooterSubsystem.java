package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.util.BSMath;

public class ShooterSubsystem extends SubsystemBase implements Lifecycle {

  private boolean enabled = false;

  private final CANSparkMax rightShooterMotor = new CANSparkMax(Constants.SHOOTERWHEELRIGHT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax followerMotor = new CANSparkMax(Constants.SHOOTERWHEELLEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax backspinMotor = new CANSparkMax(Constants.BACKSPIN_BOI_ID, MotorType.kBrushless);

  private final double DEFAULT_SHOOTER_SPEED = .3;
  private static final String SHOOTER_SPEED_KEY = "Shooter Speed";
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private final Solenoid shooterHood = new Solenoid(PneumaticsModuleType.REVPH, 3);
  //private LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);

  private boolean minimumSpeedCheckEnabled = true;

  public enum ShooterProfile {
    Short(1650), Long(2055), LowGoal(800), TarmacEdge(2200), AutoCenterEdge(1700), Downtown(2500), Dynamic(0), Off(0);

    private double value;

    private ShooterProfile(double value) {
      this.value = value;
    }
  }

  private ShooterProfile currentProfile = ShooterProfile.Off;

  private boolean closedLoop = true;
  private double targetRPM = 0.0;
  // tested 2255 for long shot
  // 2100 + hoodopen for right outside the tarmac

  public ShooterSubsystem() {
    rightShooterMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();


    followerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    followerMotor.follow(rightShooterMotor, true);

    backspinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    backspinMotor.setInverted(true);

    shooterHood.set(false);

    SmartDashboard.setDefaultNumber(SHOOTER_SPEED_KEY, DEFAULT_SHOOTER_SPEED);
    SmartDashboard.setDefaultNumber("Shooter/Profile/Short", ShooterProfile.Short.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/Long", ShooterProfile.Long.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/LowGoal", ShooterProfile.LowGoal.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/TarmacEdge", ShooterProfile.TarmacEdge.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/Downtown", ShooterProfile.Downtown.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/AutoCenterEdge", ShooterProfile.AutoCenterEdge.value);

    kP = 0.00028;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.00017;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 2500;

    // set PID coefficients
    var pidController = rightShooterMotor.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    // // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Shooter/P Gain", kP);
    SmartDashboard.putNumber("Shooter/I Gain", kI);
    SmartDashboard.putNumber("Shooter/D Gain", kD);
    SmartDashboard.putNumber("Shooter/I Zone", kIz);
    SmartDashboard.putNumber("Shooter/Feed Forward", kFF);
    SmartDashboard.putNumber("Shooter/Max Output", kMaxOutput);
    SmartDashboard.putNumber("Shooter/Min Output", kMinOutput);
    SmartDashboard.setDefaultNumber("Shooter/TargetRPM", targetRPM);

    SmartDashboard.setDefaultNumber("Shooter/Backspin/Percent", 0.0);

  }

  @Override
  public void teleopInit() {
    this.disable();
  }
 
  public boolean atMinimumSpeed() {
    if (!minimumSpeedCheckEnabled) {
      return true;
    }
    boolean retVal = false;
    if (targetRPM != 0) {
      double speedRatio = rightShooterMotor.getEncoder().getVelocity() / targetRPM;
      if (speedRatio >= .95) {
        retVal = true;
      }
    }
    return retVal;

  };

  public void disableMinimumSpeedCheck() {
    this.minimumSpeedCheckEnabled = false;
  }

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
    targetRPM = 0;
  }

  /*
   * 2496.45 RPM works for the corner shot next to the driverstation with hood
   * closed
   * 2290.86 RPM works for the shot from the launchPad with hood closed
   * 1762.2 RPM works for the short tarmac shot with hood open
   */
  @Deprecated
  public void longShot() {
    shooterHood.set(false);
  }

  @Deprecated
  public void shortShot() {
    shooterHood.set(true);
  }

  public ShooterProfile getProfile() {
    return this.currentProfile;
  }

  public void setProfile(ShooterProfile profile) {
    double rpm = 0.0;
    switch (profile) {
      case Short:
        rpm = SmartDashboard.getNumber("Shooter/Profile/Short", ShooterProfile.Short.value);
        shooterHood.set(true);
        break;
      case Long:
        rpm = SmartDashboard.getNumber("Shooter/Profile/Long", ShooterProfile.Long.value);
        shooterHood.set(false);
        break;
      case AutoCenterEdge:
        rpm = SmartDashboard.getNumber("Shooter/Profile/AutoCenterEdge", ShooterProfile.AutoCenterEdge.value);
        shooterHood.set(false);
        break;
      case Downtown:
        rpm = SmartDashboard.getNumber("Shooter/Profile/Downtown", ShooterProfile.Downtown.value);
        shooterHood.set(false);
        break;
      case Dynamic:
        var dynamicInfo = calculateDynamicRPM();
        shooterHood.set(dynamicInfo.getFirst());
        rpm = dynamicInfo.getSecond();
        break;
      case LowGoal:
        rpm = SmartDashboard.getNumber("Shooter/Profile/LowGoal", ShooterProfile.LowGoal.value);
        shooterHood.set(true);
        break;
      case TarmacEdge:
        rpm = SmartDashboard.getNumber("Shooter/Profile/TarmacEdge", ShooterProfile.TarmacEdge.value);
        shooterHood.set(true);
        break;
    }
    this.enable();
    this.currentProfile = profile;
    SmartDashboard.putNumber("Shooter/TargetRPM", rpm);
    this.targetRPM = rpm;
  }

  /**
   * Calculates the target RPM
   * @ return Pair<boolean, double> - shooter hood state and target RPM
   */
  public Pair<Boolean, Double> calculateDynamicRPM() {
    var info = Subsystems.visionSubsystem.getVisionInfo();
    if (info.distanceToTarget > 0) {
      double rpm = (100.993*java.lang.Math.sqrt(1.49073*(info.distanceToTarget)-75.6047)+623.172);
      //SmartDashboard.putNumber("Raw RPM", rpm);
     //rpm = filter.calculate(rpm);
     //SmartDashboard.putNumber("Filter RPM", rpm);
      return Pair.of(false, rpm);
     
    } else {
      // Return current state
      return Pair.of(shooterHood.get(), targetRPM);
    }
  }

  @Override
  public void periodic() {

    if (!enabled) {
      rightShooterMotor.set(0.0);
      backspinMotor.set(0.0);
      return;
    }

    if (closedLoop) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("Shooter/P Gain", 0);
      double i = SmartDashboard.getNumber("Shooter/I Gain", 0);
      double d = SmartDashboard.getNumber("Shooter/D Gain", 0);
      double iz = SmartDashboard.getNumber("Shooter/I Zone", 0);
      double ff = SmartDashboard.getNumber("Shooter/Feed Forward", 0);
      double max = SmartDashboard.getNumber("Shooter/Max Output", 0);
      double min = SmartDashboard.getNumber("Shooter/Min Output", 0);
      double rpm = SmartDashboard.getNumber("Shooter/TargetRPM", 0);

      // if PID coefficients on SmartDashboard have changed, write new values to
      var pidController = rightShooterMotor.getPIDController();
      if ((p != kP)) {
        pidController.setP(p);
        kP = p;
      }
      if ((i != kI)) {
        pidController.setI(i);
        kI = i;
      }
      if ((d != kD)) {
        pidController.setD(d);
        kD = d;
      }
      if ((iz != kIz)) {
        pidController.setIZone(iz);
        kIz = iz;
      }
      if ((ff != kFF)) {
        pidController.setFF(ff);
        kFF = ff;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        pidController.setOutputRange(min, max);
        kMinOutput = min;
        kMaxOutput = max;
      }

      // If our target RPM was overridden by the smart dashboard, update it
      if (targetRPM != rpm) {
        targetRPM = rpm;
      }

      if (currentProfile == ShooterProfile.Dynamic) {
        var dynamicInfo = calculateDynamicRPM();
        shooterHood.set(dynamicInfo.getFirst());
        targetRPM = dynamicInfo.getSecond();
      }

      targetRPM = MathUtil.clamp(targetRPM, -maxRPM, maxRPM);


      rightShooterMotor.getPIDController().setReference(targetRPM, ControlType.kVelocity);
      SmartDashboard.putNumber("Actual RPM", rightShooterMotor.getEncoder().getVelocity());

    } else {
      //
      // Handle Open Loop Control
      //
      double rightShooterSpeed = 0.0;
      if (enabled) {
        rightShooterSpeed = SmartDashboard.getNumber(SHOOTER_SPEED_KEY, DEFAULT_SHOOTER_SPEED);
        rightShooterMotor.set(rightShooterSpeed);
      }
    }

    
    double backspinPercent = SmartDashboard.getNumber("Shooter/Backspin/Percent", 0.0);
    SmartDashboard.putNumber("Shooter/Backspin/ActualRPM", backspinMotor.getEncoder().getVelocity());
    backspinMotor.set(backspinPercent);
  }
}
