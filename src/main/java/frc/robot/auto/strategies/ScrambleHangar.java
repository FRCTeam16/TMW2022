// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.commands.vision.TrackVisionTargetWithTurretCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

public class ScrambleHangar extends SequentialCommandGroup {

  private double robotAngle;
  private double driveX;
  private double driveY;

  public ScrambleHangar(double robotAngle, double driveX, double driveY) {
    this.robotAngle = robotAngle;
    this.driveX = driveX;
    this.driveY = driveY;

    addCommands(
        new InitializeAutoState(robotAngle, ShooterProfile.Short),
        pickupFirstBall(), // this one shoots befor the ball is
        shootLoad(),
        getCenterBall(),
        getRightball(),
        shootHangar());
  }

  // the first ball is a straight backup from the starting position, 1.06 meters
  // form the outer tarmac line

  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
        new WaitCommand(0.5), // wait for RPM speedup?
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull), // FIXME would rather queue
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(this.robotAngle, 0.3, driveX, driveY).withThreshold(0.03)
                .withTimeout(2.0)),
        new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.5),
        new WaitCommand(1.0),
        new InstantCommand(() -> System.out.println("****** after pickup first *****"))); // should be 1.06
  }

  private Command shootLoad() {
    return CommandGroupBase.sequence(
        new TrackVisionTargetWithTurretCommand().withTimeout(1.0),
        new InstantCommand(() -> Subsystems.feederSubsystem.pull(true))); // maybe don't pull if the wrong direction
  }

  private Command getCenterBall() {
    return CommandGroupBase.sequence(
      new ProfiledDistanceDriveCommand(-100, 0, 0, 0).withTimeout(0.5),
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(-100, .4, -2.96, -.4).withTimeout(0.5)),
        new WaitCommand(1.0),
        new ProfiledDistanceDriveCommand(80, 0, 0, 0));
  }

  private Command getRightball() {
    return CommandGroupBase.sequence(
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(80, .4, 4.16, 1.27).withTimeout(0.5)),
        new WaitCommand(1.0),
        new ProfiledDistanceDriveCommand(180, 0, 0, 0)
    );
  }

  

  private CommandGroupBase shootHangar() {
    return CommandGroupBase.sequence(
      new ProfiledDistanceDriveCommand(180, .3, 0, -1.271),
      new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.LowGoal)),
      new InstantCommand(Subsystems.feederSubsystem::pull)
    );
  }
}
