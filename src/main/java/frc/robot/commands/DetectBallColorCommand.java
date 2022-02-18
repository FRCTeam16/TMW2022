// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DetectBallColorCommand extends CommandBase {

  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher = new ColorMatch();
  private static final int BALL_DIST_THRESHOLD = 500;

  private final Color BlueBall = new Color(0.14, 0.40, 0.44);
  private final Color RedBall = new Color(0.5, 0.35, 0.12);

  /** Creates a new DetectBallColorCommand. */
  public DetectBallColorCommand(ColorSensorV3 colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSensor = colorSensor;
    this.colorMatcher.addColorMatch(BlueBall);
    this.colorMatcher.addColorMatch(RedBall);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int proximity = colorSensor.getProximity();

    
    Color detectedColor = colorSensor.getColor();
    SmartDashboard.putNumber("Color/Red", detectedColor.red);
    SmartDashboard.putNumber("Color/Blue", detectedColor.blue);
    SmartDashboard.putNumber("Color/Green", detectedColor.green);
    SmartDashboard.putNumber("Color/Proximity", proximity);

    ColorMatchResult match = colorMatcher.matchColor(detectedColor);
    String colorString = "Unknown";
    if (match != null) {
      if (match.color == BlueBall) {
        colorString = "Blue";
      } else if (match.color == RedBall) {
        colorString = "Red"; 
      }
      SmartDashboard.putNumber("Color/Detected Color Confidence", match.confidence);
    } else {
      colorString = "NO COLOR SENSOR";
    }
    SmartDashboard.putString("Color/Detected Color", colorString);

    SmartDashboard.putBoolean("Color/Ball Detected", (proximity > BALL_DIST_THRESHOLD) ? true : false);
  }

  @Override
  public boolean runsWhenDisabled() {
      return true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
