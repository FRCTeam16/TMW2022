// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.sensor.RapidReactColorMatcher;
import frc.robot.sensor.RapidReactColorMatcher.MatchedColor;

public class ShooterFeederSubsystem extends SubsystemBase {
  private boolean shooting = false;
  private boolean queuingEnabled = true;

  // Calling CANSpark motor

  private final CANSparkMax feederMotor = new CANSparkMax(Constants.SHOOTERFEEDER_MOTOR_ID, MotorType.kBrushless);
  private final double DEFAULT_FEEDER_SPEED = -.4;
  private static final String FEEDER_SPEED_KEY = "Feeder Speed";

  /** Creates a new ShooterFeederSubsystem. */
  public ShooterFeederSubsystem() {
    feederMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    SmartDashboard.setDefaultNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
  }

  public void pull() {
    if (Subsystems.shooterSubsystem.atMinimumSpeed()) {
      this.shooting = true;
    }
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
      if (Subsystems.shooterSubsystem.atMinimumSpeed()) {
        feederSpeed = SmartDashboard.getNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
      }
    } else if (queuingEnabled == true) {
      if (Subsystems.detectBallSubsystem.isBallDetected()) {
        // TODO: Shooter needs to check doesBallMatchAlliance or we signal
        feederSpeed = 0.0;
      } else {
        // no ball was detected, just run feeder until we queue a ball
        feederSpeed = SmartDashboard.getNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
      }
    }
    feederMotor.set(feederSpeed);
  }

  /**
   * Returns whether the ball matches our alliance color OR was not identified
   * 
   * @return
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
