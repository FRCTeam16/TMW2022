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
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;

public class DebugAuto extends SequentialCommandGroup {
  public DebugAuto() {
    double speed = 0.5;
    addCommands(
      CommandGroupBase.parallel(
        new InstantCommand(() -> Subsystems.intakeSubsystem.RaiseIntake()),
        new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
        new InstantCommand(() -> Subsystems.drivetrainSubsystem.setGyroOffset(180.0))
      ),
      new SimpleDistanceDriveCommand(180, speed, 0, -0.00001),
      new ProfiledDistanceDriveCommand(180, speed, 0, -4),
      new WaitCommand(2),
      new ProfiledDistanceDriveCommand(180, speed, 0, 4),
      new InstantCommand(() -> Subsystems.intakeSubsystem.DropIntake())

      //new SimpleDistanceDriveCommand(315, speed, 1, 0),
      //new SimpleDistanceDriveCommand(0, 0, 0, 0)
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
