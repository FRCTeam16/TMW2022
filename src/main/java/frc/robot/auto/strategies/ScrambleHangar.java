// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.DisableBadBallDetectionCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.commands.vision.TrackVisionTargetWithTurretCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

public class ScrambleHangar extends SequentialCommandGroup {

  private double robotAngle;
  private double driveX;
  private double driveY;

  public ScrambleHangar()  {
    this.robotAngle = 136;
    this.driveX = -1.27;
    this.driveY = 1.27;

    // Parent will run Init/pickup/shoot

    addCommands(
        new InitializeAutoState(robotAngle, ShooterProfile.Short),
        pickupFirstBall(), // this one shoots befor the ball is
        shootLoad(),
        new DisableBadBallDetectionCommand(),
        getCenterBall(),
        getRightball(),
        shootHangar()
        );
  }

  // the first ball is a straight backup from the starting position, 1.06 meters
  // form the outer tarmac line

  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
        new WaitCommand(0.5), // wait for RPM speedup?
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull), // FIXME would rather queue
            new InstantCommand(() -> Subsystems.intakeSubsystem.DropIntake()),
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(this.robotAngle, 0.3, driveX, driveY).withThreshold(0.03)
                .withTimeout(2.0)),
        new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.5),
        new InstantCommand(() -> System.out.println("****** after pickup first *****"))); // should be 1.06
  }

  private Command shootLoad() {
    return CommandGroupBase.sequence(
        new TrackVisionTargetWithTurretCommand().withTimeout(1.0),
        new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
        new WaitCommand(1.5),
        new InstantCommand(Subsystems.feederSubsystem::dontPull)
        ); // maybe don't pull if the wrong direction
  }

  private Command getCenterBall() {
    return CommandGroupBase.sequence(
      new ProfiledDistanceDriveCommand(-100, 0, 0, 0).withTimeout(0.5),
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(-100, 0.6, -0.8, -2.96).withEndSpeed(.1).withTimeout(2.0))
        //new TurnToAngleCommand(80).withTimeout(0.5)
        // new ProfiledDistanceDriveCommand(80, 0, 0, 0).withTimeout(0.5));
    );
  }

  private Command getRightball() {
    return CommandGroupBase.sequence(
        new ProfiledDistanceDriveCommand(0, 0.4, 0, 4.1).withTimeout(3.0),
        new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.LowGoal)),
        new ProfiledDistanceDriveCommand(0, 0.6, 1.7, 0).withTimeout(1.5),
        new ProfiledDistanceDriveCommand(0, 0, 0, 0).withTimeout(0.25)
    );
  }

  

  private CommandGroupBase shootHangar() {
    return CommandGroupBase.sequence(
     // new ProfiledDistanceDriveCommand(0, 0.4, -2, 0),
      new InstantCommand(() -> Subsystems.turretSubsystem.centerTurret()),
      new WaitCommand(0.1),
      new InstantCommand(Subsystems.feederSubsystem::pull),
      new WaitCommand(2.0)
    );
  }
}
