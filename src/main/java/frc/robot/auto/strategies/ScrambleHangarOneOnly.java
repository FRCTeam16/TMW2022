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

/**
 * Scramble only the hanger side ball in case our alliance can't get out of the way
 */
public class ScrambleHangarOneOnly extends SequentialCommandGroup {

  private double robotAngle;
  private double driveX;
  private double driveY;

  public ScrambleHangarOneOnly()  {
    this.robotAngle = 136;
    this.driveX = -1.27;
    this.driveY = 1.27;

    // Parent will run Init/pickup/shoot

    addCommands(
        new InitializeAutoState(robotAngle, ShooterProfile.Dynamic),
        pickupFirstBall(), // this one shoots befor the ball is
        shootLoad(),
        getWallBall(),
        shootHangar()
        );
  }

  // the first ball is a straight backup from the starting position, 1.06 meters
  // form the outer tarmac line

  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
        new WaitCommand(0.5), // wait for RPM speedup?
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull),
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
        );
  }

  private Command getWallBall() {
    double angle = 45.0;
    double zero = 0.0;
    // 180 y
    // 120 x
    return CommandGroupBase.sequence(
      new TurnToAngleCommand(angle).withTimeout(1.0),
      new InstantCommand(Subsystems.turretSubsystem::disableVisionTracking),
      new DisableBadBallDetectionCommand(),
      new ProfiledDistanceDriveCommand(angle, 0.6, 1.3, .94).withTimeout(2.0),
      new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.HangerDump)),
      new TurnToAngleCommand(zero).withTimeout(1.0)
    );
  }

  private CommandGroupBase shootHangar() {
    return CommandGroupBase.sequence(
      new InstantCommand(() -> Subsystems.turretSubsystem.centerTurret()),
      new WaitCommand(0.2),
      new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
      new WaitCommand(2.0)
    );
  }
}
