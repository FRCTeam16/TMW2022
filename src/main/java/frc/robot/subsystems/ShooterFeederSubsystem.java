package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;

public class ShooterFeederSubsystem extends SubsystemBase implements Lifecycle {
  private boolean shooting = false;

  private boolean queuingEnabled = true;
  private boolean haltWhenQueued = false;
  private static final double QUEUING_FEEDER_SPEED = -0.1;

  private final CANSparkMax feederMotor = new CANSparkMax(Constants.SHOOTERFEEDER_MOTOR_ID, MotorType.kBrushless);
  private final double DEFAULT_FEEDER_SPEED = -1;
  private static final String FEEDER_SPEED_KEY = "Feeder Speed";
  private boolean autoFeeder = false;

  public ShooterFeederSubsystem() {
    feederMotor.restoreFactoryDefaults();
    feederMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    SmartDashboard.setDefaultNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
    SmartDashboard.setDefaultNumber("Feeder/QueuingSpeed", QUEUING_FEEDER_SPEED);
    SmartDashboard.setDefaultBoolean("Feeder/QueuingEnabled", queuingEnabled);
  }

  @Override
  public void teleopInit() {
    // queuingEnabled = false;
    autoFeeder = false;
    this.dontPull();
  }

  @Override
  public void autoInit() {
    autoFeeder = true;
    // queuingEnabled = false;
  }

  public void pull(boolean override) {
    if (override || Subsystems.shooterSubsystem.atMinimumSpeed()) {
      this.shooting = true;
    }
  }

  public void pull() {
    this.pull(false);
  }

  public void dontPull() {
    this.shooting = false;
  }

  public void enableQueuing() {
    this.queuingEnabled = true;
  }

  public void disableQueuing() {
    System.out.println("[ShooterFeederSubsystem] Disabling Queuing");
    this.queuingEnabled = false;
  }

  public boolean isShooting() {
    return shooting;
  }
  
  @Override
  public void periodic() {
    double feederSpeed = 0.0;

    if (shooting) {
      haltWhenQueued = false;
      if (!autoFeeder) {
        feederSpeed = SmartDashboard.getNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
        // if (Subsystems.shooterSubsystem.atMinimumSpeed()) {
        //   feederSpeed = SmartDashboard.getNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
        // } else {
        //   System.out.println("Not at minimum speed");
        // }
      } else {
        feederSpeed = -0.5;
      }
    } else if (queuingEnabled == true) {
      if (Subsystems.detectBallSubsystem.isBallDetected()) {
        feederSpeed = 0.0;
        haltWhenQueued = true;
      } else {
        // no ball was detected, just run feeder until we queue a ball
        feederSpeed = SmartDashboard.getNumber("Feeder/QueuingSpeed", QUEUING_FEEDER_SPEED);
      }
    }

    SmartDashboard.putBoolean("Feeder/QueuingEnabled", queuingEnabled);
    SmartDashboard.putNumber("Feeder/Amps", feederMotor.getOutputCurrent());
    feederMotor.set(feederSpeed);
  }

}
