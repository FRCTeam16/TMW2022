// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class SimpleTrackTargetCommand extends CommandBase {
  private VisionSubsystem visionSubsystem;
  private DrivetrainSubsystem drivetrainSubystem;

  /** Creates a new SimpleTrackTargetCommand. */
  public SimpleTrackTargetCommand(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
    addRequirements(visionSubsystem, drivetrainSubsystem);
    this.visionSubsystem = visionSubsystem;
    this.drivetrainSubystem = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var info = visionSubsystem.getVisionInfo();
    var offset = 0.0;
    SmartDashboard.putBoolean("Vision HasTarget", info.hasTarget);
    if (true || info.hasTarget) {
      offset = info.xOffset;
      SmartDashboard.putNumber("Vision xOffset", offset);
    }
    var setpoint = drivetrainSubystem.getGyroscopeRotation().getDegrees() + offset;
    drivetrainSubystem.getRotationController().setSetpoint(setpoint);

    SmartDashboard.putNumber("STTC Setpoint", setpoint);
    var output = drivetrainSubystem.getRotationController().calculate(drivetrainSubystem.getGyroscopeRotation().getDegrees(), setpoint);
    SmartDashboard.putNumber("STTC Output", output);
    drivetrainSubystem.drive(new ChassisSpeeds(0, 0, -Math.toRadians(output)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(visionSubsystem.getVisionInfo().xOffset) < 1;
  }
}
