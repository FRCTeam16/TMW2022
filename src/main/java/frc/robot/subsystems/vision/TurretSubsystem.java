// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

   private final CANSparkMax turretMotor = new CANSparkMax(Constants.SHOOTERFEEDER_MOTOR_ID, MotorType.kBrushless);
   private final double DEFAULT_TURRET_SPEED = -.4;
   private static final String TURRET_SPEED_KEY = "Feeder Speed";

  private boolean openLoop = true;

  public TurretSubsystem() {
    turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    SmartDashboard.setDefaultNumber(TURRET_SPEED_KEY, DEFAULT_TURRET_SPEED);
  }

  @Override
  public void periodic() {
    double turretSpeed = 0.0;
    // This method will be called once per scheduler run
    if (openLoop = true) {
      turretSpeed = SmartDashboard.getNumber(TURRET_SPEED_KEY, DEFAULT_TURRET_SPEED);
    }
  }
}
