package frc.robot.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.strategies.AbstractTwoBallStrategy;

import frc.robot.auto.strategies.DebugAuto;
import frc.robot.auto.strategies.FiveBallStragety;
import frc.robot.auto.strategies.JustShoot;
import frc.robot.auto.strategies.RotateTuneStrategy;
import frc.robot.auto.strategies.SwervePathStrategy;

public class AutoManager {
    public enum AutoStrategies {
        DebugAuto, DebugTimed, DebugPath, DebugRotate, FiveBall, TwoBallRight, TwoBallHangar, TwoBallCenter, JustShoot
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Command> strategyLookup = new HashMap<>();

    public AutoManager() {
        chooser.addOption("Debug Auto", AutoStrategies.DebugAuto);
        chooser.addOption("Debug Timed", AutoStrategies.DebugTimed);
        chooser.addOption("Debug Path", AutoStrategies.DebugPath);
        chooser.addOption("Debug Rotate", AutoStrategies.DebugRotate);
        chooser.setDefaultOption("Five Ball", AutoStrategies.FiveBall);
        chooser.addOption("TwoBall Right", AutoStrategies.TwoBallRight);
        chooser.addOption("TwoBall Center", AutoStrategies.TwoBallCenter);
        chooser.addOption("TwoBall Hangar", AutoStrategies.TwoBallHangar);
        chooser.addOption("JustShoot", AutoStrategies.JustShoot);

        // chooser.addOption(name, object);
        SmartDashboard.putData(chooser);

        initializeAuto();
    }

    public void initializeAuto() {
        strategyLookup.put(AutoStrategies.FiveBall, new FiveBallStragety());
        strategyLookup.put(AutoStrategies.TwoBallRight, new AbstractTwoBallStrategy(-90, 0, -1.20));
        strategyLookup.put(AutoStrategies.TwoBallCenter, new AbstractTwoBallStrategy(-170, -1.18, -0.2));
        strategyLookup.put(AutoStrategies.TwoBallHangar, new AbstractTwoBallStrategy(135, -1.06, 1.06));
        strategyLookup.put(AutoStrategies.JustShoot, new JustShoot());
    }

    public Command getSelectedCommand() {
        Command selected = null;

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected());
        } else {
            switch (chooser.getSelected()) {
    
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

                case FiveBall:
                    selected = new FiveBallStragety();
                    break;

                default:
                    selected = new InstantCommand();
            }
        }
        return selected;
    }

}
