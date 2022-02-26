package frc.robot.auto.strategies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;
import frc.robot.commands.SimpleDistanceDriveCommand;


public class CenterAutoStrategy extends SequentialCommandGroup {

  public CenterAutoStrategy() {
    addCommands(
      initialState(),
      pickupFirstBall()
    );
  }

  private Command initialState() {
    return CommandGroupBase.parallel(
      new InstantCommand(Subsystems.drivetrainSubsystem::zeroGyroscope),
      new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d())))
    );
  }

  private Command pickupFirstBall() {
    return CommandGroupBase.sequence(
      CommandGroupBase.parallel(
        new InstantCommand(Subsystems.intakeSubsystem::DropIntake),
        new InstantCommand(Subsystems.intakeSubsystem::enable)),
      new SimpleDistanceDriveCommand(30, 0.5, -1, -0.25)
    );
  }
}
