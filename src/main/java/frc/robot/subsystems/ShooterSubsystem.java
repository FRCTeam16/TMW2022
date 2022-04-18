package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;

public class ShooterSubsystem extends SubsystemBase implements Lifecycle {

  private boolean enabled = false;

  private final CANSparkMax rightShooterMotor = new CANSparkMax(Constants.SHOOTERWHEELRIGHT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax followerMotor = new CANSparkMax(Constants.SHOOTERWHEELLEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax backspinMotor = new CANSparkMax(Constants.BACKSPIN_BOI_ID, MotorType.kBrushless);

  private final double DEFAULT_SHOOTER_SPEED = .3;
  private static final String SHOOTER_SPEED_KEY = "Shooter Speed";
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private final Solenoid shooterHood = new Solenoid(PneumaticsModuleType.REVPH, 3);
  private LinearFilter distanceFilter = LinearFilter.singlePoleIIR(0.05, 0.02);
  private boolean badBallDetectionEnabled = true;

  private boolean minimumSpeedCheckEnabled = true;

  public enum ShooterProfile {
    Short(1200), Long(2055), HangerDump(500), LowGoal(500),
    TarmacEdge(2200), AutoCenterEdge(1700), Downtown(2500), 
    Dynamic(0), Off(0);

    private double value;

    private ShooterProfile(double value) {
      this.value = value;
    }
  }

  private ShooterProfile currentProfile = ShooterProfile.Off;

  private boolean closedLoop = true;
  private double targetRPM = 0.0;
  private double lastTargetRPM = 0.0;

  // Just tracks whether we want to throttle drive speed or not, used by default drive command
  private boolean shootingDriveSpeedThrottle = false;

  private double backspinTargetRPM = 0.0;
  private double backspinP = 0.0004;
  private double backspinI = 0.0;
  private double backspinD = 0.0;
  private double backspinFF = 0.000193;

  private ShooterDynamicDistance dynamicDistance = new ShooterDynamicDistance();
  

  public ShooterSubsystem() {
    rightShooterMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();
    backspinMotor.restoreFactoryDefaults();


    followerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    followerMotor.follow(rightShooterMotor, true);

    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    followerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    backspinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    backspinMotor.setInverted(true);

    shooterHood.set(false);

    SmartDashboard.setDefaultNumber(SHOOTER_SPEED_KEY, DEFAULT_SHOOTER_SPEED);
    SmartDashboard.setDefaultNumber("Shooter/Profile/Short", ShooterProfile.Short.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/Long", ShooterProfile.Long.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/HangerDump", ShooterProfile.HangerDump.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/LowGoal", ShooterProfile.LowGoal.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/TarmacEdge", ShooterProfile.TarmacEdge.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/Downtown", ShooterProfile.Downtown.value);
    SmartDashboard.setDefaultNumber("Shooter/Profile/AutoCenterEdge", ShooterProfile.AutoCenterEdge.value);

    kP = 0.0003;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.00019;
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

    SmartDashboard.setDefaultNumber("Shooter/Backspin/Percent", backspinTargetRPM);
    SmartDashboard.setDefaultNumber("Shooter/Backspin/P Gain", backspinP);
    SmartDashboard.setDefaultNumber("Shooter/Backspin/I Gain", backspinI);
    SmartDashboard.setDefaultNumber("Shooter/Backspin/D Gain", backspinD);
    SmartDashboard.setDefaultNumber("Shooter/Backspin/Feed Forward", backspinFF);
    SmartDashboard.setDefaultNumber("Shooter/Backspin/TargetRPM", backspinTargetRPM);
    
    var backspinPID = backspinMotor.getPIDController();
    backspinPID.setP(backspinP);
    backspinPID.setI(backspinI);
    backspinPID.setD(backspinD);
    backspinPID.setFF(backspinFF);

  }

  @Override
  public void teleopInit() {
    this.disable();
    this.shootingDriveSpeedThrottle = false;
    this.enableBadBallDetection();
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
    backspinTargetRPM = 0.0;
  }

  public void enableBadBallDetection() { this.badBallDetectionEnabled = true; }
  public void disableBadBallDetection() { this.badBallDetectionEnabled = false; }

  public void enableShootingDriveSpeedThrottle() { this.shootingDriveSpeedThrottle = true; }
  public void disableShootingDriveSpeedThrottle() { this.shootingDriveSpeedThrottle = false; }
  public boolean isShootingDriveSpeedThrottle() { return this.shootingDriveSpeedThrottle; }

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
    double backRpm = 0.0;

    switch (profile) {
      case Short:
        rpm = SmartDashboard.getNumber("Shooter/Profile/Short", ShooterProfile.Short.value);
        backRpm = 2000;
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
        distanceFilter.reset(); // reset if we are coming back into Dynamic
        var dynamicInfo = calculateDynamicRPM();
        rpm = dynamicInfo.shooterRPM;
        backRpm = dynamicInfo.backspinRPM;
        break;
      case HangerDump:
        rpm = SmartDashboard.getNumber("Shooter/Profile/HangerDump", ShooterProfile.HangerDump.value);
        backRpm = 4800;
        shooterHood.set(true);
        break;
      case LowGoal:
        rpm = SmartDashboard.getNumber("Shooter/Profile/LowGoal", ShooterProfile.HangerDump.value);
        backRpm = 2000;
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
    SmartDashboard.putNumber("Shooter/Backspin/TargetRPM", backRpm);
    this.targetRPM = rpm;
    this.backspinTargetRPM = backRpm;
  }

  /**
   * Calculates the target RPM
   * @ return Pair<boolean, double> - shooter hood state and target RPM
   */
  public ShootInfo calculateDynamicRPM() {
    var shootInfo = new ShootInfo();
    var info = Subsystems.visionSubsystem.getVisionInfo();

    // if (info.distanceToTarget > 0) {
    //   var distance = distanceFilter.calculate(info.distanceToTarget);
    // }
    var distance = info.distanceToTarget;
    var range = dynamicDistance.getCurrentShootingRange(distance);

    if (info.distanceToTarget > 0) {
      // Hood Open Profiles
      if (ShooterDynamicDistance.Range.Short == range) {
        shootInfo.shooterRPM = 1200;
        shootInfo.backspinRPM = 2000;
        shootInfo.hoodOpen = true;
      }
      else if (ShooterDynamicDistance.Range.Middle == range) {
        shootInfo.shooterRPM = (.0487 * (distance * distance)) - (5.50 * distance) + 845;
        shootInfo.backspinRPM = 4800;
        shootInfo.hoodOpen = true;
      } 
      else if (ShooterDynamicDistance.Range.Long == range) {
        shootInfo.shooterRPM = (6.76 * distance) + 81;
        shootInfo.backspinRPM = 4800;
        shootInfo.hoodOpen = false;
      }
    } else {
      // Range was unknonw,  No target detected, return current state
      shootInfo.shooterRPM = targetRPM;
      shootInfo.backspinRPM = backspinTargetRPM;
      shootInfo.hoodOpen = shooterHood.get();
    }
    // System.out.println("Calculating Dynamic: " + shootInfo);
    return shootInfo;
  }

  @Override
  public void periodic() {

    if (!enabled) {
      rightShooterMotor.set(0.0);
      backspinMotor.set(0.0);
      return;
    }

    if (closedLoop) {

      // ------------------- Main PID --------------------//

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

      // ------------------- Backspin PID --------------------//

      double bp = SmartDashboard.getNumber("Shooter/Backspin/P Gain", 0);
      double bi = SmartDashboard.getNumber("Shooter/Backspin/I Gain", 0);
      double bd = SmartDashboard.getNumber("Shooter/Backspin/D Gain", 0);
      // double biz = SmartDashboard.getNumber("Shooter/Backspin/I Zone", 0);
      // double bff = SmartDashboard.getNumber("Shooter/Backspin/Feed Forward", 0);
      // double bmax = SmartDashboard.getNumber("Shooter/Backspin/Max Output", 0);
      // double bmin = SmartDashboard.getNumber("Shooter/Backspin/Min Output", 0);
      double brpm = SmartDashboard.getNumber("Shooter/Backspin/TargetRPM", 0);

     var bpidController = backspinMotor.getPIDController();
     if ((bp != backspinP)) {
      bpidController.setP(bp);
      backspinP = bp;
    }
    if ((bi != backspinI)) {
      bpidController.setI(bi);
      backspinI = bi;
    }
    if ((bd != backspinD)) {
      bpidController.setD(bd);
      backspinD = bd;
    }
    // if ((biz != kIz)) {
    //   bpidController.setIZone(iz);
    //   kIz = iz;
    // }
    // if ((bff != backspinFF)) {
    //   bpidController.setFF(bff);
    //   backspinFF = ff;
    // }
    // if ((bmax != kMaxOutput) || (bmin != kMinOutput)) {
    //   bpidController.setOutputRange(bmin, bmax);
    //   kMinOutput = bmin;
    //   kMaxOutput = bmax;
    // }

    // If our target RPM was overridden by the smart dashboard, update it
    if (backspinTargetRPM != brpm) {
      backspinTargetRPM = brpm;
    }


      // ------------------- Dynamic Override --------------------//
      if (currentProfile == ShooterProfile.Dynamic) {
        var dynamicInfo = calculateDynamicRPM();
        targetRPM = dynamicInfo.shooterRPM;
        backspinTargetRPM = dynamicInfo.backspinRPM;

        // TODO: Protect against hood oscillation
        shooterHood.set(dynamicInfo.hoodOpen);
      }

      // Finally do a check about alliance matching
      if (badBallDetectionEnabled && Subsystems.detectBallSubsystem.isEnabled() && 
          Subsystems.detectBallSubsystem.isBallDetected() && 
          !Subsystems.detectBallSubsystem.doesBallMatchAlliance()) {
        targetRPM = ShooterProfile.HangerDump.value;
      }

      // Apply safety RPMs
      targetRPM = MathUtil.clamp(targetRPM, -maxRPM, maxRPM);

      // Send data to controller
      SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
      SmartDashboard.putNumber("Shooter/Backspin/TargetRPM", backspinTargetRPM);

      // System.out.println("SHOOTER: " + targetRPM + " | BACK: " + backspinTargetRPM);
      if ( Math.abs(targetRPM - lastTargetRPM) > 1) {
        lastTargetRPM = targetRPM;
        rightShooterMotor.getPIDController().setReference(targetRPM, ControlType.kVelocity);
      }

      backspinMotor.getPIDController().setReference(backspinTargetRPM, ControlType.kVelocity);

      SmartDashboard.putNumber("Actual RPM", rightShooterMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("Actual Backspin RPM", backspinMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("Shooter/Backspin/ActualRPM", backspinMotor.getEncoder().getVelocity());
    } else {
      
      // Handle Open Loop Control
      
      double rightShooterSpeed = 0.0;
      if (enabled) {
        rightShooterSpeed = SmartDashboard.getNumber(SHOOTER_SPEED_KEY, DEFAULT_SHOOTER_SPEED);
        rightShooterMotor.set(rightShooterSpeed);

        double backspinPercent = SmartDashboard.getNumber("Shooter/Backspin/Percent", 0.0);
        SmartDashboard.putNumber("Shooter/Backspin/ActualRPM", backspinMotor.getEncoder().getVelocity());
        backspinMotor.set(backspinPercent);
      }
    }
  }


  private class ShootInfo {
    // public static final double BACKSPIN_THRESHOLD = 80;
    // public static final double HOOD_THRESHOLD = 165;

    public double shooterRPM = 0;
    public double backspinRPM = 0;
    public boolean hoodOpen = false;

    @Override
    public String toString() {
        return "ShootInfo(" + shooterRPM + ", " + backspinRPM + ", " + hoodOpen + ")";
    }
  }
}
