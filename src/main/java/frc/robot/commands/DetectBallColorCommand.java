package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

/**
 * Shows telemetry information about detected ball color
 */
public class DetectBallColorCommand extends CommandBase {


  public DetectBallColorCommand() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Subsystems.detectBallSubsystem.getDetector().telemetry();
  }

  @Override
  public boolean runsWhenDisabled() {
      return true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
