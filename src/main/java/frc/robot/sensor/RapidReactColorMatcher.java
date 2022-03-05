package frc.robot.sensor;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class RapidReactColorMatcher {

    public enum MatchedColor {
        Unknown, Blue, Red
    }

    private final ColorSensorV3 colorSensor;
    private final Color BlueBall = new Color(0.14, 0.40, 0.44);
    private final Color RedBall = new Color(0.5, 0.35, 0.12);
    private static final int BALL_DIST_THRESHOLD = 500;

    private final ColorMatch colorMatcher = new ColorMatch();

    private int proximity = -1;
    private Color detectedColor;
    private MatchedColor matchedColor;
    private double matchedConfidence;


    public RapidReactColorMatcher(ColorSensorV3 colorSensor) {
        this.colorSensor = colorSensor;
        this.colorMatcher.addColorMatch(BlueBall);
        this.colorMatcher.addColorMatch(RedBall);
    }

    public int getProximity() {
        return proximity;
    }

    public Color getDetectedColor() {
        return this.detectedColor;
    }
    public MatchedColor getMatchedColor() {
        return this.matchedColor;
    }


    public void execute() {
        SmartDashboard.putBoolean("ColorSensor/Connected", colorSensor.isConnected());
        this.proximity = colorSensor.getProximity();
        var detectedColor = this.colorSensor.getColor();
        var match = colorMatcher.matchColor(detectedColor);
        this.matchedColor = MatchedColor.Unknown;
        if (match != null) {
            this.matchedConfidence = match.confidence;
            if (match.color == BlueBall) {
                matchedColor = MatchedColor.Blue;
            } else if (match.color == RedBall) {
                matchedColor = MatchedColor.Red;
            }
        }
    }

    public void telemetry() {
        if (detectedColor != null) {
            SmartDashboard.putNumber("ColorSensor/Red", detectedColor.red);
            SmartDashboard.putNumber("ColorSensor/Blue", detectedColor.blue);
            SmartDashboard.putNumber("ColorSensor/Green", detectedColor.green);
        }
        SmartDashboard.putNumber("ColorSensor/Proximity", proximity);
        SmartDashboard.putString("ColorSensor/Detected Color", (matchedColor != null) ? matchedColor.name() : "Null Matched Color");
        SmartDashboard.putNumber("ColorSensor/Detected Color Confidence", matchedConfidence);
    }


}
