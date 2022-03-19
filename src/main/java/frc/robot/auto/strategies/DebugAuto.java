// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SimpleDistanceDriveCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.commands.testing.TimedDriveProfiledCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

public class DebugAuto extends SequentialCommandGroup {
  public DebugAuto() {
    double speed = 0.5;
    double offset = -90.0;
    addCommands(
      new InitializeAutoState(45, ShooterProfile.Dynamic),
      new ProfiledDistanceDriveCommand(45, 0.25, -1, 0).withRobotCentric().withTimeout(3.0),
      new ProfiledDistanceDriveCommand(45, 0.25, 0, -1).withRobotCentric().withTimeout(3.0)
    );
  }

  private CommandGroupBase driveSquare(double angle, double speed) {
    return CommandGroupBase.sequence(
      new ProfiledDistanceDriveCommand(angle, speed, 1, 0),
      new ProfiledDistanceDriveCommand(angle, speed, 0, 1),
      new ProfiledDistanceDriveCommand(angle, speed, -1, 0),
      new ProfiledDistanceDriveCommand(angle, speed, 0, -1)

      // new SimpleDistanceDriveCommand(0, speed, 1, 0),
      // new SimpleDistanceDriveCommand(0, speed, 0, 1),
      // new SimpleDistanceDriveCommand(0, speed, -1, 0),
      // new SimpleDistanceDriveCommand(0, speed, 0, -1)
    );
  }
}
