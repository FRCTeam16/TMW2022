package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnToAngleCommand extends TrapezoidProfileCommand {
  private int num_scans_seen = 0;

  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(double targetAngle, DrivetrainSubsystem drivetrain) {
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
          System.out.println("***** " +  state.position + " | " + state.velocity);
          SmartDashboard.putString("TTAC", state.toString());
          var output = drivetrain.getRotationController().calculate(drivetrain.getGyroscopeRotation().getDegrees(), state.position);
          drivetrain.drive(new ChassisSpeeds(0, 0, Math.toRadians(output)));
        });

        this.addRequirements(drivetrain);

  }

  @Override
  public boolean isFinished() {
    boolean retval = false;
    if (num_scans_seen >= 5) {
      retval = true;
    } else if (Subsystems.drivetrainSubsystem.getRotationController().atSetpoint()) {
      num_scans_seen++;
    } else {
      num_scans_seen = 0;
    }
      //return this.drivetrainSubsystem.getRotationController().atSetpoint();
      return retval;
  }
 
}
