// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final WPI_TalonSRX motor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
  private final double INTAKE_SPEED = -0.5;

  private boolean enabled = false;

  public void enable() {

    this.enabled = true;

  }

  public void disable() {

    this.enabled = false;

  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    double speed = 0.0;
    if (enabled) {
      speed = INTAKE_SPEED;

    }
    motor.set(speed);
  }
}
