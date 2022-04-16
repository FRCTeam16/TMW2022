package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;

public class ProfiledTurnToAngleCommand extends CommandBase {
  private double targetAngle;
  private State goal;
  private State currentState;


  public ProfiledTurnToAngleCommand(double targetAngle) {
    this.targetAngle = targetAngle;
    this.goal = new State(targetAngle, 0.0);

    addRequirements(Subsystems.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentState = new State(
      Subsystems.drivetrainSubsystem.getGyroscopeRotation().getDegrees(), 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDegrees = Subsystems.drivetrainSubsystem.getGyroscopeRotation().getDegrees();
    double output = Subsystems.drivetrainSubsystem.getRotationController().calculate(currentDegrees, this.targetAngle);
    this.currentState = new State(currentDegrees, output);

    var profile = new TrapezoidProfile(
      Constants.Auto.ThetaControllerConstraints,
      this.goal,
      this.currentState
    );

    var outputRads = Math.toRadians(profile.calculate(0.2).velocity);

    System.out.println("PTTAC: output=" + outputRads);
    Subsystems.drivetrainSubsystem.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        0, 0, outputRads, Subsystems.drivetrainSubsystem.getGyroscopeRotation())
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var finished = Subsystems.drivetrainSubsystem.getRotationController().atSetpoint();
    System.out.println("PTTAC: isFinished? " + finished);
    return finished;
  }
}
