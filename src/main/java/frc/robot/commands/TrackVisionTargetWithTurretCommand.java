package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class TrackVisionTargetWithTurretCommand extends CommandBase {
  int scansRequired = 0;
  int scanCount = 0;

  public TrackVisionTargetWithTurretCommand() {
    this(5);
  }

  public TrackVisionTargetWithTurretCommand(int scanHoldsRequired) {
    this.scansRequired = scanHoldsRequired;
    addRequirements(Subsystems.turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scanCount = 0;
    Subsystems.turretSubsystem.enableVisionTracking();
  }

  @Override
  public void execute() {
    if (Subsystems.turretSubsystem.hasVisionTarget()) {
      scanCount++;
    } else {
      scanCount = 0;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return scanCount > scansRequired;
  }
}
