package frc.robot.auto.strategies;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.ProfiledTurnToAngleCommand;
import frc.robot.commands.TurnToAngleCommand;

public class RotateTuneStrategy extends SequentialCommandGroup {
  public RotateTuneStrategy() {

    double offset = 0.0;

    addCommands(
      new InstantCommand(() -> System.out.println("TTAC")),
      new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
      new InstantCommand(() -> Subsystems.drivetrainSubsystem.zeroGyroscope()),
      new InstantCommand(() -> Subsystems.drivetrainSubsystem.setGyroOffset(offset)),

      new ProfiledTurnToAngleCommand(45),
      new WaitCommand(1.5),
      new ProfiledTurnToAngleCommand(135),
      new WaitCommand(1.5),
      new ProfiledTurnToAngleCommand(-135),
      new WaitCommand(1.5),
      new ProfiledTurnToAngleCommand(-45),
      new WaitCommand(1.5),
      new ProfiledTurnToAngleCommand(0)

      // new TurnToAngleCommand(45),
      // new WaitCommand(1.5),
      // new TurnToAngleCommand(135),
      // new WaitCommand(1.5),
      // new TurnToAngleCommand(-135),
      // new WaitCommand(1.5),
      // new TurnToAngleCommand(-45),
      // new WaitCommand(1.5),
      // new TurnToAngleCommand(0)
    );
  }
}
