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
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.commands.vision.TrackVisionTargetWithTurretCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

public class JustShoot extends SequentialCommandGroup {

  double robotAngle = 180.0;
  double driveX = -1.5;
  double driveY = 0.0;

  public JustShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      initialStateShoot(),
      shootLoad(),
      driveBack()
    );
  }

  private Command initialStateShoot() {
    
    return CommandGroupBase.parallel(
        new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
        new InstantCommand(Subsystems.drivetrainSubsystem::zeroGyroscope).andThen(
            new InstantCommand(() -> Subsystems.drivetrainSubsystem.setGyroOffset(robotAngle))), // FIXME need to find out the
                                                                                          // angle to the ball from the
                                                                                          // starting position
        new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Short)),
        new InstantCommand(Subsystems.feederSubsystem::dontPull),
        new InstantCommand(Subsystems.shooterSubsystem::enable),
        new InstantCommand(Subsystems.turretSubsystem::enableVisionTracking),
        new InstantCommand(Subsystems.intakeSubsystem::reverse),
        new InstantCommand(Subsystems.intakeSubsystem::DropIntake));
  }

  private Command shootLoad() {
    return CommandGroupBase.sequence(
        new TrackVisionTargetWithTurretCommand().withTimeout(1.0),
        new WaitCommand(2.0),
        new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
        new WaitCommand(2.0)); // maybe don't pull if the wrong direction
  }

  private Command driveBack() {
    return CommandGroupBase.sequence(
        new WaitCommand(0.5), // wait for RPM speedup?
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull), // FIXME would rather queue
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(this.robotAngle, 0.3, driveX, driveY).withThreshold(0.03).withTimeout(2.0)),
        new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.5),
        new InstantCommand(() -> System.out.println("****** after drive back *****"))); // should be 1.06
  }
}
