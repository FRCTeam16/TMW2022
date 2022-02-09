package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Limelight.SceneInfo;

/**
 * Vision subsystem.  
 * 
 * May eventually move to be a PID subsystem to handle rotation/targeting duties?
 */
public class VisionSubsystem extends SubsystemBase {
  private final Limelight limelight;
  private VisionInfo visionInfo = new VisionInfo();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    limelight = new Limelight();
  }

  public Limelight getLimelight() {
    return limelight;
  }

  public VisionInfo getVisionInfo() {
    return visionInfo;
  }

  @Override
  public void periodic() {
    var sceneInfo = limelight.getScene();
    
    // Update out information
    visionInfo.hasTarget = sceneInfo.hasTarget;
    visionInfo.xOffset = sceneInfo.xOffset;
    visionInfo.yOffset = sceneInfo.yOffset;
    visionInfo.targetArea = sceneInfo.targetArea;
  }

  /**
   * Calculates the distance to the currently observed target, or -1 if no target
   * 
   * @param heightToCamera 
   * @param heightToTarget
   * @return
   */
  public double CalculateDistance(double heightToCamera, double heightToTarget) {
    if (this.visionInfo.hasTarget) {
      return (heightToTarget - heightToCamera) / Math.tan(this.visionInfo.yOffset);
    } else {
      return -1;
    }
}

  /**
   * Vision Information
   */
  public static class VisionInfo extends SceneInfo {}
}
