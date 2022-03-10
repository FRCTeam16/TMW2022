// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import javax.sound.sampled.SourceDataLine;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SimpleDistanceDriveCommand extends CommandBase {

  private static final double THRESHOLD = 0.05;

  private double angle;
  private double speed;
  private double xdist;
  private double ydist;
  private Translation2d targetPose;

  public SimpleDistanceDriveCommand(double angleDegrees, double speed, double xInches, double yInches) {
    this.angle = angleDegrees;
    this.speed = speed;
    this.xdist = xInches;
    this.ydist = yInches;

    addRequirements(Subsystems.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var currentPose = Subsystems.drivetrainSubsystem.getPose().getTranslation();
    var translation = new Translation2d(xdist, ydist);
    var xy_trans = currentPose.plus(translation);
    this.targetPose = new Translation2d(xy_trans.getX(), xy_trans.getY());

    System.out.println("****************> SDDC Initialize");
    System.out.println("Current Pose: " + currentPose);
    System.out.println("Target Pose : " + targetPose);
    System.out.println("<**************** SDDC Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentPose = Subsystems.drivetrainSubsystem.getPose().getTranslation();

    var driveAngle = Math.atan2(
      targetPose.getY() - currentPose.getY(),
      targetPose.getX() - currentPose.getX());

    double vxMetersPerSecond = Math.abs(speed * Math.cos(driveAngle));
    double vyMetersPerSecond = Math.abs(speed * Math.sin(driveAngle));

    if (currentPose.getY() > targetPose.getY()) {
      vyMetersPerSecond = -vyMetersPerSecond;
    }
    if (currentPose.getX() > targetPose.getX()) {
      vxMetersPerSecond = -vxMetersPerSecond;
    }

    var twist = Subsystems.drivetrainSubsystem.getRotationController().calculate(
        Subsystems.drivetrainSubsystem.getGyroscopeRotation().getDegrees(), angle);
    var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vxMetersPerSecond * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
        vyMetersPerSecond * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        Math.toRadians(twist), 
        Subsystems.drivetrainSubsystem.getGyroscopeRotation());
    Subsystems.drivetrainSubsystem.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var distance = Subsystems.drivetrainSubsystem.getPose().getTranslation().getDistance(targetPose);
    System.out.println("DIST: " + distance + " | T:" + targetPose + " | C:" + Subsystems.drivetrainSubsystem.getPose());
    return (distance < THRESHOLD);
  }
}
