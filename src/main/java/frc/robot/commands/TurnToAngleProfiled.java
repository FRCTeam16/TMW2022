package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToAngleProfiled extends ProfiledPIDCommand {

  private static final double kP = 7.5;
  private static final double kI = 3.0;
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
              DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND, 
              DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARED)),
        // This should return the measurement
        () -> drivetrain.getGyroscopeRotation().getDegrees(),
        // This should return the goal (can also be a constant)
        targetAngle,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          SmartDashboard.putNumber("TurnTOAngleProfiled", output);
          drivetrain.drive(new ChassisSpeeds(0, 0, output * Math.PI / 180.0));
        },
        drivetrain);

      getController().enableContinuousInput(-180, 180);
      getController().setTolerance(1);
      //getController().setIntegratorRange(minimumIntegral, maximumIntegral);
      SmartDashboard.putNumber("TTAP Goal", getController().getGoal().position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
