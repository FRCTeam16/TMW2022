// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private boolean enabled = false;

  private final WPI_TalonSRX intakeMotor = Constants.Objects.INTAKE_MOTOR;
  private final double DEFAULT_INTAKE_SPEED = -0.5;
  private static final String INTAKE_SPEED_KEY = "Intake Speed";

  private final WPI_TalonSRX beaterMotor = new WPI_TalonSRX(Constants.BEATER_MOTOR_ID);
  private final double DEFAULT_BEATERBAR_SPEED = -1;
  private static final String BEATERBAR_SPEED_KEY = "Beater Bar Speed";

  // Calling CANSpark motor 
  /*
   * private final CANSparkMax beaterMotor = new
   * CANSparkMax(Constants.BEATER_MOTOR_ID, MotorType.kBrushless); private final
   * double DEFAULT_BEATERBAR_SPEED = 0.5; private static final String
   * BEATERBAR_SPEED_KEY = "Beater Bar Speed";
   */


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    SmartDashboard.setDefaultNumber(INTAKE_SPEED_KEY, DEFAULT_INTAKE_SPEED);
    SmartDashboard.setDefaultNumber(BEATERBAR_SPEED_KEY, DEFAULT_BEATERBAR_SPEED);
  }

  public void enable() {
    this.enabled = true;
  }

  public void disable() {
    this.enabled = false;
  }

  @Override
  public void periodic() {
    double intakeSpeed = 0.0;
    double beaterSpeed = 0.0;
    if (enabled) {
      intakeSpeed = SmartDashboard.getNumber(INTAKE_SPEED_KEY, DEFAULT_INTAKE_SPEED);
      beaterSpeed = SmartDashboard.getNumber(BEATERBAR_SPEED_KEY, DEFAULT_BEATERBAR_SPEED);
    }
    intakeMotor.set(intakeSpeed);
    beaterMotor.set(beaterSpeed);
  }
}
