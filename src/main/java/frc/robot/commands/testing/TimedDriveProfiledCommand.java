package frc.robot.commands.testing;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TimedDriveProfiledCommand extends TrapezoidProfileCommand {
  /** Creates a new TimedDriveProfiledCommand. */
  public TimedDriveProfiledCommand(double xSpeed, double ySpeed, double targetAngle) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 5,
                DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND / 2),
            // Goal state
            new TrapezoidProfile.State(),
            // Initial state
            new TrapezoidProfile.State()),
        state -> {
          double x = xSpeed * state.velocity;
          double y = ySpeed * state.velocity;
          Subsystems.drivetrainSubsystem.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  x, 
                  y,
                  Math.toRadians(Subsystems.drivetrainSubsystem.getRotationOutput(targetAngle)),
                  Subsystems.drivetrainSubsystem.getGyroscopeRotation()));
        });
  }
}
