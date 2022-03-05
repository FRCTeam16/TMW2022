package frc.robot.commands.prefs;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.BSPrefs;

public class ZeroWheelOffsets extends CommandBase {
  public ZeroWheelOffsets() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var prefs = BSPrefs.getOffsetsInstance();
    prefs.setDouble("FLOFF", 0.0);
    prefs.setDouble("FROFF", 0.0);
    prefs.setDouble("RLOFF", 0.0);
    prefs.setDouble("RROFF", 0.0);
    prefs.savePreferences();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
