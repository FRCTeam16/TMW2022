// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.prefs;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.BSPrefs;

public class SaveWheelOffsets extends CommandBase {
  /** Creates a new SaveWheelOffsets. */
  public SaveWheelOffsets() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    var prefs = BSPrefs.getOffsetsInstance();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    var drivetrain = inst.getTable("Shuffleboard").getSubTable("Drivetrain");

    prefs.setDouble("FLOFF", drivetrain.getSubTable("Front Left Module").getEntry("Absolute Encoder Angle").getDouble(0.0));
    prefs.setDouble("FROFF", drivetrain.getSubTable("Front Right Module").getEntry("Absolute Encoder Angle").getDouble(0.0));
    prefs.setDouble("RLOFF", drivetrain.getSubTable("Back Left Module").getEntry("Absolute Encoder Angle").getDouble(0.0));
    prefs.setDouble("RROFF", drivetrain.getSubTable("Back Right Module").getEntry("Absolute Encoder Angle").getDouble(0.0));

    prefs.savePreferences();
  }

  @Override
  public boolean runsWhenDisabled() {
      return true;
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
