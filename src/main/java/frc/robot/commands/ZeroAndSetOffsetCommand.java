// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class ZeroAndSetOffsetCommand extends CommandBase {
  private double offsetAngleDegrees;

  /** Creates a new ZeroAndSetOffsetCommand. */
  public ZeroAndSetOffsetCommand(double offsetAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.offsetAngleDegrees = offsetAngleDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.drivetrainSubsystem.zeroGyroscope();
    Subsystems.drivetrainSubsystem.setGyroOffset(offsetAngleDegrees);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
