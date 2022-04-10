package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.sensor.RapidReactColorMatcher;
import frc.robot.sensor.RapidReactColorMatcher.MatchedColor;

public class ShooterFeederSubsystem extends SubsystemBase implements Lifecycle {
  private boolean shooting = false;
  private boolean queuingEnabled = false;

  private final CANSparkMax feederMotor = new CANSparkMax(Constants.SHOOTERFEEDER_MOTOR_ID, MotorType.kBrushless);
  private final double DEFAULT_FEEDER_SPEED = -1;
  private static final String FEEDER_SPEED_KEY = "Feeder Speed";
  private boolean autoFeeder = false;

  public ShooterFeederSubsystem() {
    feederMotor.restoreFactoryDefaults();
    feederMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    SmartDashboard.setDefaultNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
  }

  @Override
  public void teleopInit() {
    queuingEnabled = false;
    autoFeeder = false;
    this.dontPull();
  }

  @Override
  public void autoInit() {
    autoFeeder = true;
    queuingEnabled = false;
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
    this.queuingEnabled = false;
  }

  @Override
  public void periodic() {
    double feederSpeed = 0.0;

    if (shooting) {
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
        // TODO: Shooter needs to check doesBallMatchAlliance or we should signal them
        feederSpeed = 0.0;
      } else {
        // no ball was detected, just run feeder until we queue a ball
        feederSpeed = SmartDashboard.getNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
      }
    }

    SmartDashboard.putNumber("Feeder/Amps", feederMotor.getOutputCurrent());
    // System.out.println("Setting feeder speed: " + feederSpeed + " | shooting: " + shooting + " | autoFeeder: " + autoFeeder);
    feederMotor.set(feederSpeed);
  }

  /**
   * Returns whether the ball matches our alliance color OR was not identified
   */
  public boolean doesBallMatchAlliance() {
    RapidReactColorMatcher.MatchedColor allianceColor = MatchedColor.Unknown;

    if (DriverStation.getAlliance() == Alliance.Red) {
      allianceColor = MatchedColor.Red;

    } else if (DriverStation.getAlliance() == Alliance.Blue) {
      allianceColor = MatchedColor.Blue;
    }

    if (allianceColor == MatchedColor.Unknown ||
        allianceColor == Subsystems.detectBallSubsystem.getDetector().getMatchedColor()) {
      return true;
    }

    return false;
  }
}
