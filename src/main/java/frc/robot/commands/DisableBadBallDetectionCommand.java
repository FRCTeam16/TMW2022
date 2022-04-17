package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class DisableBadBallDetectionCommand extends CommandBase {
  /** Creates a new DisableBadBallDetectionCommand. */
  public DisableBadBallDetectionCommand() {
    addRequirements(Subsystems.turretSubsystem, Subsystems.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.turretSubsystem.disableBadBallDetection();
    Subsystems.shooterSubsystem.disableBadBallDetection();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
