package frc.robot.subsystems.vision;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.VisionSubsystem.VisionInfo;

public class TurretSubsystem extends SubsystemBase {
   private final CANSparkMax turretMotor = new CANSparkMax(99, MotorType.kBrushless);
   private final double DEFAULT_TURRET_SPEED = -.4;
   private final double VISION_THRESHOLD = 1.0;

   enum RunState {
     OpenLoop, Vision
   }
   private RunState runState = RunState.OpenLoop;
   private boolean direction = true;


  public TurretSubsystem() {
    turretMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    SmartDashboard.setDefaultNumber("Turret/Open/DefaultSpeed", DEFAULT_TURRET_SPEED);
    SmartDashboard.setDefaultNumber("Turret/Vision/Threshold", VISION_THRESHOLD);
  }

  public void openForward() {
    runState = RunState.OpenLoop;
    direction = true;
  }

  public void openBackwards() {
    runState = RunState.OpenLoop;
    direction = false;
  }

  @Override
  public void periodic() {
    double defaultSpeed = SmartDashboard.getNumber("Turret/Open/DefaultSpeed", DEFAULT_TURRET_SPEED);
    double speed = 0.0;
    if (runState == RunState.OpenLoop) {
      speed = defaultSpeed * ((direction) ? 1 : -1);
      turretMotor.set(speed);
    } else if (runState == RunState.Vision) {
      double threshold = SmartDashboard.getNumber("Turret/Vision/Threshold", VISION_THRESHOLD);
      VisionInfo info = Subsystems.visionSubsystem.getVisionInfo();
      if (info.hasTarget && Math.abs(info.xOffset) > threshold) {
        // Could do PID control here
        // Set speed based on offset direction
        if (info.xOffset < 0) {
          speed = -speed;
        }
      }
    }
    // Both using openloop
    turretMotor.set(speed);
  }
}
