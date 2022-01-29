package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToAngleProfiled extends ProfiledPIDCommand {

  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;


  /** Creates a new TurnToAngleProfiled. */
  public TurnToAngleProfiled(double targetAngle, DrivetrainSubsystem drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            kP,
            kI,
            kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARED, 
              DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARED)),
        // This should return the measurement
        () -> drivetrain.getGyroscopeRotation().getDegrees(),
        // This should return the goal (can also be a constant)
        targetAngle,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.drive(new ChassisSpeeds(0, 0, output * Math.PI / 180.0));
        },
        drivetrain);

      getController().enableContinuousInput(-180, 180);
      //  FIXME: Test this
      //getController().setTolerance(5.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
