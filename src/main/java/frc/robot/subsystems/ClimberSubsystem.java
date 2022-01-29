// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  public enum ClimberState {
    kClimb, kHold, kExtend, kDisabled
  }

  private ClimberState currentState = ClimberState.kDisabled;

  private final WPI_TalonFX climberMotor1 = new WPI_TalonFX(Constants.CLIMBER_MOTOR_ID);
  private final double DEFAULT_CLIMB_SPEED = -0.35;
  private static final String CLIMB_SPEED_KEY = "Climber Speed";

  private final double DEFAULT_EXTEND_SPEED = 0.20;
  private static final String EXTEND_SPEED_KEY = "Extend Speed";

  private final double DEFAULT_HOLD_SPEED = -0.15;
  private static final String HOLD_SPEED_KEY = "Hold Speed";

  private final double DEFAULT_DISABLED_SPEED = 0;
  private static final String DISABLED_SPEED_KEY = "DISABLED Speed";

  public ClimberSubsystem() {
    SmartDashboard.setDefaultNumber(CLIMB_SPEED_KEY, DEFAULT_CLIMB_SPEED);
    SmartDashboard.setDefaultNumber(EXTEND_SPEED_KEY, DEFAULT_EXTEND_SPEED);
    SmartDashboard.setDefaultNumber(HOLD_SPEED_KEY, DEFAULT_HOLD_SPEED);
    SmartDashboard.setDefaultNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);

    CommandScheduler.getInstance().schedule(new CommandBase() {
      @Override
      public void execute() {
        SmartDashboard.putNumber("Climber Encder", climberMotor1.getSelectedSensorPosition());
        SmartDashboard.putString("Climber State", currentState.name());

      }

      @Override
      public boolean runsWhenDisabled() {
        return true;

      }
    });

  }

  public void setClimberState(ClimberState state) {

    this.currentState = state;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double climbSpeed = SmartDashboard.getNumber(CLIMB_SPEED_KEY, DEFAULT_CLIMB_SPEED);
    double holdSpeed = SmartDashboard.getNumber(HOLD_SPEED_KEY, DEFAULT_HOLD_SPEED);
    double extendSpeed = SmartDashboard.getNumber(EXTEND_SPEED_KEY, DEFAULT_EXTEND_SPEED);
    double defaultSpeed = SmartDashboard.getNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);

    final double climberOutput;

    switch (currentState) {
      case kClimb:
        climberOutput = climbSpeed;
        break;

      case kHold:
        climberOutput = holdSpeed;
        break;

      case kExtend:
        climberOutput = extendSpeed;
        break;

      case kDisabled:
        climberOutput = defaultSpeed;
        break;

      default:
        climberOutput = 0.0;

    }

    climberMotor1.set(climberOutput);
  }
}
