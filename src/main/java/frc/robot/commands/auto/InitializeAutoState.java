package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems;
import frc.robot.subsystems.ShooterSubsystem.ShooterProfile;

public class InitializeAutoState extends ParallelCommandGroup {

  /** Creates a new InitializeAutoState. */
  public InitializeAutoState(double initialRobotAngleDegrees, ShooterProfile initialProfile) {
    addCommands(
      new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))), // would be nice to have real pose info
      new InstantCommand(Subsystems.drivetrainSubsystem::zeroGyroscope).andThen(
          new InstantCommand(() -> Subsystems.drivetrainSubsystem.setGyroOffset(initialRobotAngleDegrees))),
      new InstantCommand(() -> Subsystems.shooterSubsystem.setProfile(initialProfile)),
      new InstantCommand(Subsystems.feederSubsystem::dontPull),
      new InstantCommand(Subsystems.shooterSubsystem::enable),
      new InstantCommand(Subsystems.turretSubsystem::enableVisionTracking),
      new InstantCommand(Subsystems.intakeSubsystem::DropIntake));
  }
}
