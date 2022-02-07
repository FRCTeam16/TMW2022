package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToAngleCommand extends TrapezoidProfileCommand {

  private DrivetrainSubsystem drivetrainSubsystem;

  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(double targetAngle, DrivetrainSubsystem drivetrain) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND, 
              DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARED),
            // Goal state
            new TrapezoidProfile.State(targetAngle, 0),
            // Initial state
            new TrapezoidProfile.State(drivetrain.getGyroscopeRotation().getDegrees(), 0)),
        state -> {
          // Use current trajectory state here
          var output = drivetrain.getRotationController().calculate(drivetrain.getGyroscopeRotation().getDegrees(), state.position);
          var outputRads = output * Math.PI / 180.0;
          drivetrain.drive(new ChassisSpeeds(0, 0, outputRads));
        });
        this.drivetrainSubsystem = drivetrain;
        this.addRequirements(drivetrain);
  }

  @Override
  public boolean isFinished() {
      return this.drivetrainSubsystem.getRotationController().atSetpoint();
  }
 
}
