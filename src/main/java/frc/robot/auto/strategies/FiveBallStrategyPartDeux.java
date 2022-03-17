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

public class FiveBallStrategyPartDeux extends SequentialCommandGroup {
  /** Creates a new FiveBallStragety. */
  public FiveBallStrategyPartDeux() {

    addCommands(
        new InitializeAutoState(-90.0, ShooterProfile.Short),
        pickupFirstBallAndReturn(),
        shootLoad(2.0),
        pickupSecondBall(),
        shootLoad(1.5),
        pickupThirdBall(),
        finishAuto()
        );
  }

  private Command pickupFirstBallAndReturn() {
    double robotAngle = -90.0;
    double speed = 0.3;
    double driveX = 0;
    double driveY = -1.20;

    return CommandGroupBase.sequence(
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(robotAngle, speed, driveX, driveY).withThreshold(0.03).withTimeout(2.0)),
        // stop(robotAngle),
        // new WaitCommand(0.1),
        new ProfiledDistanceDriveCommand(robotAngle, speed, -driveX, -driveY).withThreshold(0.03).withTimeout(2.0),
        stop(robotAngle)
    );
  }


  private Command shootLoad(double shootTime) {
    return CommandGroupBase.sequence(
        new TrackVisionTargetWithTurretCommand().withTimeout(0.25),
        new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
        new WaitCommand(shootTime), // maybe don't pull if the wrong direction
        new InstantCommand(Subsystems.feederSubsystem::dontPull)
    );
  }

  
  private Command pickupSecondBall() {
    double robotAngle = -180.0;
    double speed = 0.5;
    double distX = -2.95;
    double distY = 0.0;

    return CommandGroupBase.sequence(
      new InstantCommand(() -> System.out.println("==> pickupSecondBall")),
      new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Dynamic)),
      new ProfiledDistanceDriveCommand(robotAngle, speed, distX, distY),
      stop(robotAngle)
    );
  }


  // hypotenuce to the tarmac ball is 4.2 meters
  private Command pickupThirdBall() {
    double robotAngle = -170.0;
    double speed = 1.0;
    double distX = -4.5;
    double distY = -1.0;

    return CommandGroupBase.sequence(
      // new ProfiledDistanceDriveCommand(robotAngle, speed, 0, -0.1).withTimeout(0.5),
      // new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.25),
      
      new ProfiledDistanceDriveCommand(robotAngle, speed, distX, distY).withThreshold(0.03),
      stop(robotAngle),
      new WaitCommand(0.5),
      new ProfiledDistanceDriveCommand(robotAngle, speed, -(distX/2), -distY),
      stop(robotAngle)
    );
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

  private Command stop(double robotAngle) {
    return new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.1);
  }

}