package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

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
    // output = MathUtil.clamp(output, -0.5, 0.5);
    var outputRads = Math.toRadians(output);

    System.out.println("TTAC: output=" + outputRads);
    Subsystems.drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, outputRads, Subsystems.drivetrainSubsystem.getGyroscopeRotation())
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("TTAC: isFinished? " + Subsystems.drivetrainSubsystem.getRotationController().atSetpoint());
    return Subsystems.drivetrainSubsystem.getRotationController().atSetpoint();
  }
}
