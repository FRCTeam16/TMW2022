package frc.robot.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.strategies.AbstractTwoBallStrategy;

import frc.robot.auto.strategies.DebugAuto;
import frc.robot.auto.strategies.FiveBallStragety;
import frc.robot.auto.strategies.FiveBallStrategyPartDeux;
import frc.robot.auto.strategies.JustShoot;
import frc.robot.auto.strategies.RotateTuneStrategy;
import frc.robot.auto.strategies.SwervePathStrategy;
import frc.robot.auto.strategies.ShootFirstBall5Ball;

public class AutoManager {
    public enum AutoStrategies {
        DebugAuto, DebugTimed, DebugPath, DebugRotate, 
        FiveBall, TwoBallRight, TwoBallHangar, TwoBallCenter, JustShoot,
        FiveBallStrategyPartDeux,ShootFirstBall5Ball
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Command> strategyLookup = new HashMap<>();

    public AutoManager() {
        chooser.addOption("Debug Auto", AutoStrategies.DebugAuto);
        chooser.addOption("Debug Timed", AutoStrategies.DebugTimed);
        chooser.addOption("Debug Path", AutoStrategies.DebugPath);
        chooser.addOption("Debug Rotate", AutoStrategies.DebugRotate);
        chooser.addOption("Five Ball", AutoStrategies.FiveBall);
        chooser.addOption("TwoBall Right", AutoStrategies.TwoBallRight);
        chooser.addOption("TwoBall Center", AutoStrategies.TwoBallCenter);
        chooser.addOption("TwoBall Hangar", AutoStrategies.TwoBallHangar);
        chooser.addOption("JustShoot", AutoStrategies.JustShoot);
        chooser.addOption("5BS Part Deux", AutoStrategies.FiveBallStrategyPartDeux);
        chooser.setDefaultOption("First Ball 5 Ball", AutoStrategies.ShootFirstBall5Ball);


        // chooser.addOption(name, object);
        SmartDashboard.putData(chooser);

        initializeAuto();
    }

    public void initializeAuto() {
        strategyLookup.put(AutoStrategies.FiveBall, new FiveBallStragety());
        strategyLookup.put(AutoStrategies.TwoBallRight, new AbstractTwoBallStrategy(-90, 0, -1.20));
        strategyLookup.put(AutoStrategies.TwoBallCenter, new AbstractTwoBallStrategy(-170, -1.77, -0.31));
        strategyLookup.put(AutoStrategies.TwoBallHangar, new AbstractTwoBallStrategy(135, -1.27, 1.27));
        strategyLookup.put(AutoStrategies.JustShoot, new JustShoot());
        strategyLookup.put(AutoStrategies.FiveBallStrategyPartDeux, new FiveBallStrategyPartDeux());
        strategyLookup.put(AutoStrategies.ShootFirstBall5Ball, new ShootFirstBall5Ball());
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

    public void showSelectedAuto() {
        var selected = chooser.getSelected();
        SmartDashboard.putString("Selected Auto", (selected != null) ? selected.name() : "Unknown" );
    }

}
