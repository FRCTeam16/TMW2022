
package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToDistanceProfiled extends ProfiledPIDCommand {

  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  /** Creates a new DriveToDistanceProfiled. */
  
  public DriveToDistanceProfiled(double distanceInMeters, DrivetrainSubsystem drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            kP,
            kI,
            kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)),
        // This should return the measurement
        () -> drivetrain.getPose().getX(),
        // This should return the goal (can also be a constant)
        () -> drivetrain.getPose().transformBy(
          new Transform2d(
            new Translation2d(distanceInMeters, 0.0), 
            new Rotation2d())).getX(),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
              output, 
              0, //
              0, 
              drivetrain.getGyroscopeRotation()));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
