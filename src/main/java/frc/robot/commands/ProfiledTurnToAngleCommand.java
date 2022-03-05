package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.DrivetrainSubsystem;

@Deprecated
public class ProfiledTurnToAngleCommand extends TrapezoidProfileCommand {
  private int num_scans_seen = 0;

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
          var outputPct = drivetrain.getRotationController().calculate(currentRotation.getDegrees(), state.position);
          var output = outputPct;
          // var output = RotationController.clampToDPS(outputPct);
          // outputPct = MathUtil.clamp(outputPct, -0.6, 0.6);
          // var output = outputPct * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;
          // return MathUtil.clamp(outputPercent, -0.6, 0.6) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_DEGREES_PER_SECOND;

          System.out.println("[TTAC] S.P= " +  state.position + " | S.V=" + state.velocity + " | R=" + currentRotation + " | O= " + output + " | OP= " + outputPct);

          drivetrain.drive(
            (fieldCentric) ? 
              ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Math.toRadians(outputPct), currentRotation):
              new ChassisSpeeds(0, 0, Math.toRadians(output)));
        });
  }

  @Override
  public boolean isFinished() {
    boolean retval = false;
    // if (num_scans_seen >= 5) {
    //   retval = true;
    // } else if (Subsystems.drivetrainSubsystem.getRotationController().atSetpoint()) {
    //   num_scans_seen++;
    // } else {
    //   num_scans_seen = 0;
    // }
      return Subsystems.drivetrainSubsystem.getRotationController().atSetpoint();
      // return retval;
  }
 
}
