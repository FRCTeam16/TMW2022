// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.SimpleDistanceDriveCommand;

public class DebugAuto extends SequentialCommandGroup {
  public DebugAuto() {
    double speed = 0.75;
    addCommands(
      new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),

      new SimpleDistanceDriveCommand(45, speed, 1, 0),

      new SimpleDistanceDriveCommand(315, speed, 1, 0),

      new SimpleDistanceDriveCommand(0, 0, 0, 0)
    );
  }

  private CommandGroupBase driveSquare(double speed) {
    return CommandGroupBase.sequence(
      new SimpleDistanceDriveCommand(0, speed, 1, 0),
      new SimpleDistanceDriveCommand(0, speed, 0, 1),
      new SimpleDistanceDriveCommand(0, speed, -1, 0),
      new SimpleDistanceDriveCommand(0, speed, 0, -1)
    );
  }
}
