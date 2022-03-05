package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensor.RapidReactColorMatcher;

public class DetectBallSubsystem extends SubsystemBase {
  private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);
  public final RapidReactColorMatcher detector = new RapidReactColorMatcher(colorSensor);

  private final double DISTANCE_THRESHOLD = 500;
  
  public DetectBallSubsystem() {}

  @Override
  public void periodic() {
    detector.execute();    
  }

  public boolean isBallDetected() {
    return this.detector.getProximity() > DISTANCE_THRESHOLD;
  }

  public Color getDetectedColor() {
    return this.detector.getDetectedColor();
  }

  public RapidReactColorMatcher getDetector() {
    return this.detector;
  }

  public ColorSensorV3 getColorSensor() {
    return this.colorSensor;
  }
}
