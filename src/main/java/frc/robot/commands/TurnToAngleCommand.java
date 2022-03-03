package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.DrivetrainSubsystem;

// TODO: just use an internal PID

public class TurnToAngleCommand extends CommandBase {
  private double targetAngle;

  public TurnToAngleCommand(double targetAngle) {
    this.targetAngle = targetAngle;
    addRequirements(Subsystems.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDegrees = Subsystems.drivetrainSubsystem.getGyroscopeRotation().getDegrees();
    double output = Subsystems.drivetrainSubsystem.getRotationController().calculate(currentDegrees, this.targetAngle);
    // output = MathUtil.clamp(output, -0.6, 0.6);
    output *= DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    System.out.println("TTAC: output=" + output);
    Subsystems.drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, Math.toRadians(output), Subsystems.drivetrainSubsystem.getGyroscopeRotation())
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Subsystems.drivetrainSubsystem.getRotationController().atSetpoint();
  }
}
