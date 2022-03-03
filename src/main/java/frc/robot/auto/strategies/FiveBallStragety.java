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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SimpleDistanceDriveCommand;
import frc.robot.commands.ProfiledTurnToAngleCommand;
import frc.robot.commands.ZeroAndSetOffsetCommand;
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallStragety extends SequentialCommandGroup {
  /** Creates a new FiveBallStragety. */
  public FiveBallStragety() {

    addCommands(
        initialState(),
        pickupFirstBall(),
        // shootFirstLoad(),
        pickupSecondBall()
        // pickupThirdBall(),
        // shootSecondLoad(),
        // finishAuto());
        );
  }

  private Command initialState() {
    return CommandGroupBase.parallel(
        // new InstantCommand(Subsystems.drivetrainSubsystem.set)
        new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
        // new ZeroAndSetOffsetCommand(-90)
        new InstantCommand(Subsystems.drivetrainSubsystem::zeroGyroscope).andThen(
        new InstantCommand(() -> Subsystems.drivetrainSubsystem.setGyroOffset(-90)))
        //new InstantCommand(()-> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Short)),
        //new InstantCommand(Subsystems.shooterSubsystem::enable),
        //new InstantCommand(Subsystems.intakeSubsystem::DropIntake),
        );
  }

  // the first ball is a straight backup from the starting position, 1.06 meters
  // form the outer tarmac line
  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
        new ProfiledDistanceDriveCommand(-115, 0, 0, -0.1).withTimeout(1.0),
        //  new TurnToAngleCommand(-115, Subsystems.drivetrainSubsystem).withTimeout(0.5),
        CommandGroupBase.parallel(
            new ProfiledDistanceDriveCommand(-115, 0, 0, 0).withTimeout(0.5),
           // new InstantCommand(Subsystems.feederSubsystem::pull),
            new WaitCommand(4.0)),
        CommandGroupBase.parallel(
          new InstantCommand(() -> System.out.println("****** doing pickup *****")),
            //new InstantCommand(Subsystems.feederSubsystem::dontPull),
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(-90, 0.3, 0, -1.16)
            .withThreshold(0.03).withTimeout(2.0)),
          new InstantCommand(() -> System.out.println("****** stopping drive *****")),
          new ProfiledDistanceDriveCommand(-115, 0, 0, 0).withTimeout(0.5),
          new InstantCommand(() -> System.out.println("****** after pickup first *****"))); // should be 1.06
  }
  
  // hypotenuce to the second ball is 3.09 meters from the first cargo location
  // can't do this one until we run to see positioning on the robot after
  // shooting.
  private Command pickupSecondBall() {
    return CommandGroupBase.sequence(
      new InstantCommand(() -> System.out.println("****** pickupSecondBall *****")),
        CommandGroupBase.parallel(
           new InstantCommand(Subsystems.intakeSubsystem::enable)),
        new ProfiledDistanceDriveCommand(141, 0.5, -2.54, 1.76) // FIXME

    );
  }

  private Command shootFirstLoad() {
    return CommandGroupBase.sequence(
        new ProfiledDistanceDriveCommand(0, .5, -1.1, 1.55),
        new InstantCommand(Subsystems.feederSubsystem::pull));
  }


  // hypotenuce to the tarmac ball is 4.2 meters
  private Command pickupThirdBall() {
    return CommandGroupBase.sequence(
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.intakeSubsystem::enable)),
        new ProfiledDistanceDriveCommand(30, 0.5, -3.9, -0.25));
  }

  private Command shootSecondLoad() {
    return CommandGroupBase.sequence(
        new InstantCommand(Subsystems.feederSubsystem::pull));
  }

  /*
   * finishAuto() is so we can have the robot sit at the tarmac for the human
   * player to feed a ball to if we want it to until auto is complete
   */
  private Command finishAuto() {
    return CommandGroupBase.sequence(
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new InstantCommand(Subsystems.feederSubsystem::pull)));
  }
}