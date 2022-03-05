package frc.robot.auto.strategies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.TrackVisionTargetWithTurretCommand;
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

public class AbstractTwoBallStrategy extends SequentialCommandGroup {

    private double robotAngle;
    private double driveX;
    private double driveY;

    public AbstractTwoBallStrategy(double robotAngle, double driveX, double driveY) {
        this.robotAngle = robotAngle;
        this.driveX = driveX;
        this.driveY = driveY;

        addCommands(
            initialStateShoot(), // this shoots ball before it leaves tarmac
            initialStateDontShoot(), // this waits for the
            pickupFirstBall(), // this one shoots befor the ball is
            shootLoad()
        );
    }

    // this will be our initial state if we want to shoot from the tarmac
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
        new InstantCommand(Subsystems.intakeSubsystem::DropIntake));
  }

  /**
   * Waits until second ball to shoot
   * @return
   */
  private Command initialStateDontShoot() {
    return CommandGroupBase.parallel(
        new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
        new InstantCommand(Subsystems.drivetrainSubsystem::zeroGyroscope).andThen(
            new InstantCommand(() -> Subsystems.drivetrainSubsystem.setGyroOffset(robotAngle))), // FIXME need to find out the
                                                                                          // angle to the ball from the
                                                                                          // starting position
        new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Short)),
        new InstantCommand(Subsystems.shooterSubsystem::enable),
        new InstantCommand(Subsystems.intakeSubsystem::DropIntake));
  }

  // the first ball is a straight backup from the starting position, 1.06 meters
  // form the outer tarmac line

  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
        new TrackVisionTargetWithTurretCommand().withTimeout(3.0),
        //new ProfiledDistanceDriveCommand(-115, 0, 0, -0.1).withTimeout(1.0),
        new WaitCommand(0.5), // wait for RPM speedup?
        // new TurnToAngleCommand(-115,
        // Subsystems.drivetrainSubsystem).withTimeout(0.5),
        CommandGroupBase.parallel(
            // new ProfiledDistanceDriveCommand(-115, 0, 0, 0).withTimeout(0.5),
            new InstantCommand(Subsystems.feederSubsystem::pull),
            new WaitCommand(2.5)),
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull), // FIXME would rather queue
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(this.robotAngle, 0.3, driveX, driveY).withThreshold(0.03).withTimeout(2.0)),
        new ProfiledDistanceDriveCommand(-115, 0, 0, 0).withTimeout(0.5),
        new InstantCommand(() -> System.out.println("****** after pickup first *****"))); // should be 1.06
  }

  private Command shootLoad() {
    return CommandGroupBase.sequence(
        // new ProfiledDistanceDriveCommand(160, .3, -1, 1),
        new InstantCommand(Subsystems.feederSubsystem::pull)); // maybe don't pull if the wrong direction
  }
}
