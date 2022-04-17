package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.WaitUntilMatchTimeCommand;
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.commands.vision.TrackVisionTargetWithTurretCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

/**
 * Setup on right tarmac to allow alliance member to run 5 ball.  We will move 
 * to block possible opponent incursion into our area and scramble the ball there.
 */
public class BlockOpponentOneBallStrategy extends SequentialCommandGroup {

  private double pickupAngle = -70.0;
  private double shootAngle = -35.0;
  
  public BlockOpponentOneBallStrategy() {
    addCommands(
      new WaitCommand(3.0), // initial delay
      new InitializeAutoState(-140.0, ShooterProfile.Dynamic),
      moveToTopOfTarmac(),
      moveForward(),
      shootBall(1.0),
      pickupOppo(),
      scrambleOppo()
    );
  }

  private Command moveToTopOfTarmac() {
    return new ProfiledDistanceDriveCommand(pickupAngle, 0.5, 2.0, -0.8);
  }

  private Command moveForward() {
    return new ProfiledDistanceDriveCommand(pickupAngle, 0.2, 0.0, -0.8);
  }

  private Command shootBall(double shootTime) {
    return CommandGroupBase.sequence(
        new TrackVisionTargetWithTurretCommand().withTimeout(0.25),
        new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
        new WaitCommand(shootTime),
        new InstantCommand(Subsystems.feederSubsystem::dontPull)
    );
  }

  private Command pickupOppo() {
    return CommandGroupBase.sequence(
      new InstantCommand(Subsystems.intakeSubsystem::enable),
      new ProfiledDistanceDriveCommand(pickupAngle, 0.2, 0.6, -1.2).withTimeout(2.0),
      new ProfiledDistanceDriveCommand(pickupAngle, 0, 0, 0).withTimeout(0.25),
      new WaitCommand(1.0)
    );
  }

  private Command scrambleOppo() {
    return CommandGroupBase.sequence(
      CommandGroupBase.parallel(
        new WaitCommand(0.5),
        new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(ShooterProfile.LowGoal)),
        new InstantCommand(Subsystems.intakeSubsystem::disable)
      ),
      new InstantCommand(Subsystems.turretSubsystem::disableBadBallDetection),
      new InstantCommand(Subsystems.turretSubsystem::disableVisionTracking),
      new InstantCommand(Subsystems.turretSubsystem::centerTurret),
      new InstantCommand(Subsystems.intakeSubsystem::RaiseIntake),
      new TurnToAngleCommand(shootAngle).withTimeout(1.5),
      new ProfiledDistanceDriveCommand(shootAngle, 0, 0, 0).withTimeout(0.25),
      new WaitUntilMatchTimeCommand(12.0),
      new InstantCommand(() -> Subsystems.feederSubsystem.pull(true)),
      new WaitCommand(2.0),
      new InstantCommand(Subsystems.feederSubsystem::dontPull)
    );
  }

}
