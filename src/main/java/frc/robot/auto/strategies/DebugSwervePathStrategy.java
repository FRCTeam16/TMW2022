package frc.robot.auto.strategies;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Subsystems;

public class DebugSwervePathStrategy extends SequentialCommandGroup {
  /** Creates a new SwervePathStrategy. */
  public DebugSwervePathStrategy() {
    addCommands(
        new InstantCommand(() -> Subsystems.drivetrainSubsystem.resetOdometry(new Pose2d())),
        generateCommand());
  }

  private TrajectoryConfig createTrajectoryConfig() {
    TrajectoryConfig config = new TrajectoryConfig(
        Constants.Auto.MaxSpeedMetersPerSecond,
        Constants.Auto.MaxAccelerationMetersPerSecondSquared)
            .setKinematics(Subsystems.drivetrainSubsystem.getSwerveDriveKinematics());
    return config;
  }

  private Command generateCommand() {
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.Auto.ThetaP, 0.0, 0.0,
        Constants.Auto.ThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand command = new SwerveControllerCommand(
      createSTrajectory(),
        Subsystems.drivetrainSubsystem::getPose,
        Subsystems.drivetrainSubsystem.getSwerveDriveKinematics(),
        new PIDController(Constants.Auto.Px, 0, 0),
        new PIDController(Constants.Auto.Py, 0, 0),
        thetaController,
        Subsystems.drivetrainSubsystem::setSwerveModuleStates,
        Subsystems.drivetrainSubsystem);

    return command.andThen(() -> Subsystems.drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0, 0, Subsystems.drivetrainSubsystem.getGyroscopeRotation())));
  }

  private Trajectory createSTrajectory() {
    Trajectory path = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d(0)),
        createTrajectoryConfig());
    return path;
  }

  private Trajectory createStraightTrajectory() {
    Trajectory path = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(2, 0, new Rotation2d(0)),
        createTrajectoryConfig());
    return path;
  }
}
