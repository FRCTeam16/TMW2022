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
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightWallAuto extends SequentialCommandGroup {
  /** Creates a new RightWallAuto. */
  public RightWallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        initialState(),
        pickUpFirstBall(),
        shootLoad());
  }

  private Command initialState() {
    return CommandGroupBase.parallel(
        // new InstantCommand(Subsystems.drivetrainSubsystem.set)
        new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
        // new ZeroAndSetOffsetCommand(-90)
        new InstantCommand(Subsystems.drivetrainSubsystem::zeroGyroscope).andThen(
            new InstantCommand(() -> Subsystems.drivetrainSubsystem.setGyroOffset(-90))),
        new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Short)),
        new InstantCommand(Subsystems.feederSubsystem::dontPull),
        new InstantCommand(Subsystems.shooterSubsystem::enable),
        new InstantCommand(Subsystems.intakeSubsystem::DropIntake));
  }

  private Command pickUpFirstBall() {
    return CommandGroupBase.sequence(
        new ProfiledDistanceDriveCommand(-115, 0, 0, -0.1).withTimeout(1.0),
        new WaitCommand(0.5), // wait for RPM speedup?
        // new TurnToAngleCommand(-115,
        // Subsystems.drivetrainSubsystem).withTimeout(0.5),
        CommandGroupBase.parallel(
            new ProfiledDistanceDriveCommand(-115, 0, 0, 0).withTimeout(0.5),
            new InstantCommand(Subsystems.feederSubsystem::pull),
            new WaitCommand(2.5)),
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull), // FIXME would rather queue
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(-90, 0.3, 0, -1.20).withThreshold(0.03).withTimeout(2.0)),
        new ProfiledDistanceDriveCommand(-115, 0, 0, 0).withTimeout(0.5),
        new InstantCommand(() -> System.out.println("****** after pickup first *****"))); // should be 1.06
  }

  private Command shootLoad() {
    return CommandGroupBase.sequence(
        new ProfiledDistanceDriveCommand(160, .3, -1, 1),
        new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Short)),
        new InstantCommand(Subsystems.feederSubsystem::pull)); // maybe don't pull if the wrong direction
  }
}
