package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj.DriverStation;
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
        new InitializeAutoState(-90.0, ShooterProfile.Dynamic),
        pickupFirstBallAndReturn(),
        pickupIntakeAndMoveBack(),
        shootLoad(2.0),
        // pickupSecondBall(),
        pickupSecondBallAngle(),  
        shootLoad(1.5),
        pickupThirdBall(),
        finishAuto()
        );
  }

  private Command pickupFirstBallAndReturn() {
    double robotAngle = -90.0;
    double speed = 1.0;
    double driveX = 0;
    double driveY = -1.4;

    return CommandGroupBase.sequence(
        CommandGroupBase.parallel(
            new InstantCommand(Subsystems.intakeSubsystem::enable),
            new ProfiledDistanceDriveCommand(robotAngle, speed, driveX, driveY)
              .withEndSpeed(0.2)
              .withThreshold(0.02)
              .withTimeout(3.0)
        ),
        new InstantCommand(() -> System.out.println("**** after move: " + DriverStation.getMatchTime())),
        stop(robotAngle).withTimeout(0.1),
        new InstantCommand(() -> System.out.println("**** after stop: " + DriverStation.getMatchTime()))
        // new WaitCommand(0.1),
        /*
        new ProfiledDistanceDriveCommand(robotAngle, speed, -driveX, -driveY).withThreshold(0.03).withTimeout(2.0),
        stop(robotAngle)
        */
        
    );
  }

  private Command pickupIntakeAndMoveBack() {
    double robotAngle = -90.0;
    double speed = 1.0;
    double driveX = 0;
    double driveY = -0.25;
    return CommandGroupBase.sequence(
      new WaitCommand(0.75),
      new InstantCommand(Subsystems.intakeSubsystem::disable),
      new InstantCommand(Subsystems.intakeSubsystem::RaiseIntake),
      CommandGroupBase.parallel(
        new ProfiledDistanceDriveCommand(robotAngle, speed, driveX, driveY)
              .withEndSpeed(0.1)
              .withThreshold(0.02)
              .withTimeout(2.0)
      ),
      stop(robotAngle)
    );
  }


  private Command shootLoad(double shootTime) {
    return CommandGroupBase.sequence(
        new InstantCommand(() -> System.out.println("**** shootLoad: " + DriverStation.getMatchTime())),
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

  private Command pickupSecondBallAngle() {
    double robotAngle = 140.0;
    double speed = 1.0;
    double distX = -3.3;
    double distY = 1.2;

    return CommandGroupBase.sequence(
      CommandGroupBase.parallel(
        new InstantCommand(() -> System.out.println("==> pickupSecondBall")),
        new InstantCommand(Subsystems.intakeSubsystem::DropIntake),
        new InstantCommand(Subsystems.intakeSubsystem::enable),
        new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.Dynamic))
      ),
      new ProfiledDistanceDriveCommand(robotAngle, speed, distX, distY).withEndSpeed(0.2).withTimeout(3),
      stop(robotAngle),
      // Turn to next
      new ProfiledDistanceDriveCommand(180.0, 0.2, -0.25, -0.25).withTimeout(0.5),
      stop(180.0)
    );
  }


  // hypotenuce to the tarmac ball is 4.2 meters
  private Command pickupThirdBall() {
    double robotAngle = -170.0;
    double speed = 1.0;
    double distX = -4.25;  // -4.5
    double distY = -0.25;  // -1.0

    return CommandGroupBase.sequence(
      // new ProfiledDistanceDriveCommand(robotAngle, speed, 0, -0.1).withTimeout(0.5),
      // new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.25),
      
      new ProfiledDistanceDriveCommand(robotAngle, speed, distX, distY).withEndSpeed(0.2).withThreshold(0.03),
      stop(robotAngle),
      new WaitCommand(0.5),
      new ProfiledDistanceDriveCommand(robotAngle, speed, -(distX), -distY).withEndSpeed(0.2)
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