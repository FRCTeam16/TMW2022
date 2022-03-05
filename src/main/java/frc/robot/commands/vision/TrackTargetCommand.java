package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;


@Deprecated
public class TrackTargetCommand extends PIDCommand {

  private static final double kP = 7.5;
  private static final double kI = 3.0;
  private static final double kD = 0.0;
  private final VisionSubsystem visionSubsystem;


  /** Creates a new TrackTargetCommand. */
  public TrackTargetCommand(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> drivetrainSubsystem.getGyroscopeRotation().getDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> {
          double offset = 0.0;
          var info = visionSubsystem.getVisionInfo();
          if (info.hasTarget) {
            offset = info.xOffset;
          }
          var setpoint = drivetrainSubsystem.getGyroscopeRotation().getDegrees() + offset; 
          SmartDashboard.putNumber("VisionTrackSet", setpoint);
          return setpoint;
        },
        // This uses the output
        output -> {
          SmartDashboard.putNumber("VisionTrackOut", output);
          drivetrainSubsystem.drive(
            new ChassisSpeeds(
              0,
              0,
              -Math.toRadians(output)
            )
          );
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(visionSubsystem, drivetrainSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-180, 180);

    this.visionSubsystem = visionSubsystem;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return visionSubsystem.getVisionInfo().hasTarget == true && 
      Math.abs(visionSubsystem.getVisionInfo().xOffset) < 1.0;
  }
}
