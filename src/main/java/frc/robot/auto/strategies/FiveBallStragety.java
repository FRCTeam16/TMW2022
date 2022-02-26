// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;
import frc.robot.commands.SimpleDistanceDriveCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallStragety extends SequentialCommandGroup {
  /** Creates a new FiveBallStragety. */
  public FiveBallStragety() {
    
    addCommands(
      initialState(),
      pickupFirstBall(),
      //shootFirstLoad(),
      pickupSecondBall()

      //
    );
    
  }

  private Command initialState() {
    return CommandGroupBase.parallel(
      new InstantCommand(Subsystems.drivetrainSubsystem::zeroGyroscope),
      new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d())))
    );
  }
  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
      CommandGroupBase.parallel(
        new InstantCommand(Subsystems.intakeSubsystem::DropIntake),
        new InstantCommand(Subsystems.intakeSubsystem::enable)),
      new SimpleDistanceDriveCommand(30, 0.5, -1, -0.25)
    );
  }
  private Command pickupSecondBall(){
    return CommandGroupBase.sequence(
      CommandGroupBase.parallel(
        new InstantCommand(Subsystems.intakeSubsystem::enable)),
      new SimpleDistanceDriveCommand(30, 0.5, -1, -0.25)
      
    );
  }
}
