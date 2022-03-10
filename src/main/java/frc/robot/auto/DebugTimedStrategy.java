package frc.robot.auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.SimpleTimedDriveCommand;
public class DebugTimedStrategy extends SequentialCommandGroup {
  /** Creates a new DebugStrategy. */
  public DebugTimedStrategy() {
    addCommands(
        new SimpleTimedDriveCommand(1, 0.25, 0, 0, Subsystems.drivetrainSubsystem),
        new SimpleTimedDriveCommand(1, 0, 0.25, 0, Subsystems.drivetrainSubsystem),
        new SimpleTimedDriveCommand(1, -0.25, 0, 0, Subsystems.drivetrainSubsystem),
        new SimpleTimedDriveCommand(1, 0, -0.25, 0, Subsystems.drivetrainSubsystem));
  }
}
