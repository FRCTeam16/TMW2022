package frc.robot.auto;

import java.util.Dictionary;
import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.strategies.CenterAutoStrategy;
import frc.robot.auto.strategies.DebugAuto;
import frc.robot.auto.strategies.RotateTuneStrategy;
import frc.robot.auto.strategies.SwervePathStrategy;

public class AutoManager {
    public enum AutoStrategies {
        Center, DebugAuto, DebugTimed, DebugPath, DebugRotate
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Command> strategyLookup = new HashMap<>();

    public AutoManager() {
        chooser.addOption("Debug Auto", AutoStrategies.DebugAuto);
        chooser.addOption("Debug Timed", AutoStrategies.DebugTimed);
        chooser.addOption("Debug Path", AutoStrategies.DebugPath);
        chooser.addOption("Debug Rotate", AutoStrategies.DebugRotate);
        chooser.addOption("Center", AutoStrategies.Center);
        // chooser.addOption(name, object);
        SmartDashboard.putData(chooser);

        initializeAuto();
    }

    public void initializeAuto() {
        strategyLookup.put(AutoStrategies.Center, new CenterAutoStrategy());
    }

    public Command getSelectedCommand() {
        Command selected = null;

        // TODO: eventually move to eager creation during disabled init of strategies
        // for potential performance
        // instantiating here allows easier re-runs during dev

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected());
        } else {
            switch (chooser.getSelected()) {
                case Center:
                    selected = new CenterAutoStrategy();
                    break;
    
                case DebugAuto:
                    selected = new DebugAuto();
                    break;
    
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
        }
        return selected;
    }

}
