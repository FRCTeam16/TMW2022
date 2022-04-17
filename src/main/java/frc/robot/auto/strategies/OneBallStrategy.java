package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.testing.ProfiledDistanceDriveCommand;
import frc.robot.commands.vision.TrackVisionTargetWithTurretCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

public class OneBallStrategy extends SequentialCommandGroup {

  private final double DELAY_START = 10.0;

  public OneBallStrategy() {
    double robotAngle = 180.0;
    double driveX = -1.77;
    double driveY = 0.0;
    
    addCommands(
      new InitializeAutoState(robotAngle, ShooterProfile.Dynamic),
      new WaitCommand(DELAY_START),
      CommandGroupBase.parallel(
        new InstantCommand(Subsystems.feederSubsystem::dontPull),
        new InstantCommand(Subsystems.intakeSubsystem::disable),

      new ProfiledDistanceDriveCommand(robotAngle, 0.3, driveX, driveY)
        .withThreshold(0.03).withTimeout(2.0)),
        
      // stop
      new ProfiledDistanceDriveCommand(robotAngle, 0, 0, 0).withTimeout(0.5),
      new TrackVisionTargetWithTurretCommand().withTimeout(1.0),
      new InstantCommand(() -> Subsystems.feederSubsystem.pull(true))
    );
  }
}
