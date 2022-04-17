package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensor.RapidReactColorMatcher;

public class DetectBallSubsystem extends SubsystemBase implements Lifecycle {
  private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kMXP);
  public final RapidReactColorMatcher detector = new RapidReactColorMatcher(colorSensor);
  
  public DetectBallSubsystem() {}

  @Override
  public void periodic() {
    detector.execute();    
  }

  public boolean isBallDetected() {
    return this.detector.isBallDetected();
  }

  public Color getDetectedColor() {
    return this.detector.getDetectedColor();
  }

  public boolean doesBallMatchAlliance() {
    return this.detector.doesBallMatchAlliance();
  }

  public RapidReactColorMatcher getDetector() {
    return this.detector;
  }

  public ColorSensorV3 getColorSensor() {
    return this.colorSensor;
  }

public boolean isEnabled() {
    return this.colorSensor.isConnected();
}
}
