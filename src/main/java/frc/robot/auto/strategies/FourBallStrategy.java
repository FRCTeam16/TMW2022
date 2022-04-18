package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.commands.vision.TrackVisionTargetWithTurretCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

/**
 * Four ball stratgy from right-side tarmac. Skips far-right ball.
 */
public class FourBallStrategy extends SequentialCommandGroup {
  
  private double initialAngle = -154.0;

  //-170, -1.77, -0.31

  public FourBallStrategy() {
    addCommands(
      new InitializeAutoState(initialAngle, ShooterProfile.Dynamic),
      pickupFirstBall(),
      shootLoad(2.0),
      pickupThirdAndFourthBalls(),
      RollToShootingPosition(),
      shootLoad(4.0)
    );
  }

  private Command shootLoad(double shootTime) {
    return CommandGroupBase.sequence(
      new TrackVisionTargetWithTurretCommand().withTimeout(0.25),
      new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
      new WaitCommand(shootTime),
      new InstantCommand(Subsystems.feederSubsystem::dontPull)
    );
  }

  private Command pickupFirstBall() {
    double driveX = -1.77;
    double driveY = -0.75;

    return CommandGroupBase.sequence(
      new WaitCommand(0.5), // wait for RPM speedup?
      CommandGroupBase.parallel(
          new InstantCommand(Subsystems.feederSubsystem::dontPull),
          new InstantCommand(Subsystems.intakeSubsystem::enable),
          new ProfiledDistanceDriveCommand(this.initialAngle, 0.3, driveX, driveY).withThreshold(0.03).withTimeout(2.0)
        ),
      stop(this.initialAngle)
    );
  }

  private Command pickupThirdAndFourthBalls() {
    double robotAngle = initialAngle;
    double speed = 1.0;
    double distX = -4.7;
    double distY = -1.5;

    double distX2 = 0.5;
    double distY2 = 0.35;

    return CommandGroupBase.sequence(
      new ProfiledDistanceDriveCommand(robotAngle, speed, distX, distY).withEndSpeed(0.2).withThreshold(0.03).withTimeout(5.0),
      stop(robotAngle),
      new ProfiledDistanceDriveCommand(robotAngle, speed, distX2, distY2).withThreshold(0.05).withTimeout(3.0),
      new WaitCommand(2.0)
    );
  }

  private Command RollToShootingPosition() {
    double robotAngle = initialAngle;
    double speed = 1.0;
    double distX = 2.5;
    double distY = 0.45;
    return CommandGroupBase.sequence(
      new ProfiledDistanceDriveCommand(robotAngle, speed, distX, distY)
        .withThreshold(0.05).withTimeout(5.0)
    );
  }


  private Command stop(double robotAngle) {
    return new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.1);
  }
 
}
