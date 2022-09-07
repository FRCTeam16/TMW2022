package frc.robot.auto.strategies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class AbstractTwoBallStrategy extends SequentialCommandGroup {

    private double robotAngle;
    private double driveX;
    private double driveY;

    public AbstractTwoBallStrategy(double robotAngle, double driveX, double driveY) {
        this.robotAngle = -90;
        this.driveX = 0;
        this.driveY = -1.2;

        addCommands(
            new InitializeAutoState(robotAngle, ShooterProfile.Dynamic),
            pickupFirstBall(), // this one shoots befor the ball is
            shootLoad()
        );
    }
 
  // the first ball is a straight backup from the starting position, 1.06 meters
  // form the outer tarmac line

  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
        new WaitCommand(0.5), // wait for RPM speedup?
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.feederSubsystem::dontPull), // FIXME would rather queue
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(this.robotAngle, 0.3, driveX, driveY).withThreshold(0.03).withTimeout(2.0)),
        new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.5),
        new WaitCommand(1.0),
        new ProfiledDistanceDriveCommand(this.robotAngle, 0.3, -driveX, -driveY).withThreshold(0.03).withTimeout(2.0),
        new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.5),
        new InstantCommand(() -> System.out.println("****** after pickup first *****"))); // should be 1.06
  }

  private Command shootLoad() {
    return CommandGroupBase.sequence(
        new TrackVisionTargetWithTurretCommand().withTimeout(1.0),
        new InstantCommand(() -> Subsystems.feederSubsystem.pull(true))); // maybe don't pull if the wrong direction
  }
}
