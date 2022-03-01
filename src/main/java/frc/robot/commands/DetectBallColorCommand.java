package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensor.RapidReactColorMatcher;

public class DetectBallColorCommand extends CommandBase {

  private final RapidReactColorMatcher colorSensor;

  public DetectBallColorCommand(RapidReactColorMatcher colorSensor) {
    this.colorSensor = colorSensor;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.colorSensor.telemetry();
  }

  @Override
  public boolean runsWhenDisabled() {
      return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
