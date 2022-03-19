package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.DrivetrainSubsystem;

@Deprecated /* Use TurnToAngleCommand */
public class ProfiledTurnToAngleCommand extends TrapezoidProfileCommand {

  public ProfiledTurnToAngleCommand(double targetAngle) {
    this(targetAngle, Subsystems.drivetrainSubsystem);
    addRequirements(Subsystems.drivetrainSubsystem);
  }

  public ProfiledTurnToAngleCommand(double targetAngle, DrivetrainSubsystem drivetrain) {
    this(targetAngle, true, drivetrain);
  }

  public ProfiledTurnToAngleCommand(double targetAngle, boolean fieldCentric, DrivetrainSubsystem drivetrain) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND / 2, 
              DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARED / 2 ),
            // Goal state
            new TrapezoidProfile.State(targetAngle, 0),
            // Initial state
            new TrapezoidProfile.State(drivetrain.getGyroscopeRotation().getDegrees(), 0)),
        state -> {
          // Use current trajectory state here
          var currentRotation = drivetrain.getGyroscopeRotation();
          var outputDeg = drivetrain.getRotationController().calculate(currentRotation.getDegrees(), state.position);
          var outputRad = Math.toRadians(outputDeg);

          System.out.println("[TTAC] S.P= " +  state.position + 
            " | S.V=" + state.velocity + 
            " | R=" + currentRotation +
            " | OR=" + outputRad +
            " | OD= " + outputDeg);

          drivetrain.drive(
            (fieldCentric) ? 
              ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, outputRad, currentRotation):
              new ChassisSpeeds(0, 0, outputRad));
        });
  }

  @Override
  public boolean isFinished() {
      return Subsystems.drivetrainSubsystem.getRotationController().atSetpoint();
  }
 
}
