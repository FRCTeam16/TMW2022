package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.strategies.RotateTuneStrategy;
import frc.robot.auto.strategies.SwervePathStrategy;
import frc.robot.commands.TurnToAngleCommand;

public class AutoManager {
    public enum AutoStrategies {
        DebugTimed, DebugPath, DebugRotate
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();

    public AutoManager() {
        chooser.setDefaultOption("Debug Timed", AutoStrategies.DebugTimed);
        chooser.addOption("Debug Path", AutoStrategies.DebugPath);
        chooser.addOption("Debug Rotate", AutoStrategies.DebugRotate);
        // chooser.addOption(name, object);
        SmartDashboard.putData(chooser);
    }

    public Command getSelectedCommand() {
        Command selected = null;

        // TODO: eventually move to eager creation during disabled init of strategies
        // for potential performance
        // instantiating here allows easier re-runs during dev
        switch (chooser.getSelected()) {
            case DebugTimed:
                selected = new DebugTimedStrategy();
                break;

            case DebugPath:
                selected = new SwervePathStrategy();
                break;

            case DebugRotate:
                selected = new RotateTuneStrategy();
                break;

            default:
                selected = new InstantCommand();
        }
        return selected;
    }

}
