// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
import frc.robot.commands.vision.TrackVisionTargetWithTurretCommand;
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
        shootLoad(),
        pickupSecondBall(),
        pickupThirdBall(),
        finishAuto()
        );
  }

  private Command initialState() {
    return CommandGroupBase.parallel(
      new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
      new InstantCommand(Subsystems.drivetrainSubsystem::zeroGyroscope).andThen(
          new InstantCommand(() -> Subsystems.drivetrainSubsystem.setGyroOffset(-90))), // FIXME need to find out the
                                                                                        // angle to the ball from the
                                                                                        // starting position
      new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.TarmacEdge)),
      new InstantCommand(Subsystems.feederSubsystem::dontPull),
      new InstantCommand(Subsystems.shooterSubsystem::enable),
      new InstantCommand(Subsystems.turretSubsystem::enableVisionTracking),
      new InstantCommand(Subsystems.intakeSubsystem::DropIntake));
  }

  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
        new WaitCommand(0.5), // wait for RPM speedup?
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull), // FIXME would rather queue
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(-90, 0.3, 0, -1.20).withThreshold(0.03).withTimeout(2.0)),
        new ProfiledDistanceDriveCommand(-90, 0, 0, 0).withTimeout(0.5),
        new InstantCommand(() -> System.out.println("****** after pickup first *****"))); // should be 1.06
  }

  private Command shootLoad() {
    return CommandGroupBase.sequence(
        new TrackVisionTargetWithTurretCommand().withTimeout(1.0),
        new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
        new WaitCommand(2.5)); // maybe don't pull if the wrong direction
  }

  
  
  // hypotenuce to the second ball is 3.09 meters from the first cargo location
  // can't do this one until we run to see positioning on the robot after
  // shooting.
  private Command pickupSecondBall() {
    return CommandGroupBase.sequence(
      new InstantCommand(() -> System.out.println("****** pickupSecondBall *****")),
      new InstantCommand(Subsystems.feederSubsystem::dontPull),
      new ProfiledDistanceDriveCommand(141, 0.5, -3.05, 0.6),  // -2.54, 1.76
      new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.AutoCenterEdge)),
      new WaitCommand(0.5),

      // Turn to third ball
      new ProfiledDistanceDriveCommand(-170, 1, 0, -0.1).withTimeout(0.5),
      new TrackVisionTargetWithTurretCommand().withTimeout(1.0),
      new ProfiledDistanceDriveCommand(-170, 0.1, 0, -0.1).withTimeout(0.25),

      // Shoot balls
      CommandGroupBase.parallel(
            new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
            new WaitCommand(1.5)), // FIXME Adjust me with the wait
      
      new InstantCommand(Subsystems.feederSubsystem::dontPull)
    );
  }

  // hypotenuce to the tarmac ball is 4.2 meters
  private Command pickupThirdBall() {
    return CommandGroupBase.sequence(
      new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Downtown)),
      // new ProfiledDistanceDriveCommand(-170, 0.5, -4.2, -0.3));   // 0.5, -4.2, -0.3
      new ProfiledDistanceDriveCommand(-170, 1.9, -4.2, -0.05));   // 0.5, -4.2, -0.3
  }

   /*
   * finishAuto() is so we can have the robot sit at the tarmac for the human
   * player to feed a ball to if we want it to until auto is complete
   */
  private Command finishAuto() {
    return CommandGroupBase.sequence(
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new InstantCommand(() -> Subsystems.feederSubsystem.pull(true))));
  }

    // the first ball is a straight backup from the starting position, 1.06 meters
  // form the outer tarmac line
  private Command XXXpickupFirstBall() {
    return CommandGroupBase.sequence(
        new ProfiledDistanceDriveCommand(-115, 0, 0, -0.1).withTimeout(1.0),
        new WaitCommand(0.5), // wait for RPM speedup?
        //  new TurnToAngleCommand(-115, Subsystems.drivetrainSubsystem).withTimeout(0.5),
        CommandGroupBase.parallel(
          new ProfiledDistanceDriveCommand(-115, 0, 0, -0.1).withTimeout(0.5),
          new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
          new WaitCommand(2.5)),  //FIXME Adjust me with the wait
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull), // FIXME would rather queue
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(-90, 0.3, 0, -1.20).withThreshold(0.03).withTimeout(2.0)),
          new ProfiledDistanceDriveCommand(-115, 0, 0, 0).withTimeout(0.5),
          new InstantCommand(() -> System.out.println("****** after pickup first *****"))); // should be 1.06
  }

  private Command XXXshootFirstLoad() {
    return CommandGroupBase.sequence(
       new InstantCommand(Subsystems.turretSubsystem::enableVisionTracking),
        new ProfiledDistanceDriveCommand(0, .5, -1.1, 1.55),
        new InstantCommand(Subsystems.feederSubsystem::pull));
  }

  private Command XXXshootSecondLoad() {
    return CommandGroupBase.sequence(
      new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Short)),
        new InstantCommand(Subsystems.feederSubsystem::pull));
  }

 
}