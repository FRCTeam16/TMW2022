package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SimpleTimedDriveCommand extends CommandBase {
  private final Timer timer = new Timer();
  private double timeToRunSeconds;
  private final double xSpeed;
  private final double ySpeed;
  private final double targetAngle;
  private final DrivetrainSubsystem drivetrain;
  
  /**
   * 
   * @param timeToRunSeconds
   * @param xSpeed -1 to 1
   * @param ySpeed -1 to 1
   * @param targetAngle
   * @param drivetrain
   */
  public SimpleTimedDriveCommand(double timeToRunSeconds, double xSpeed, double ySpeed, double targetAngle, DrivetrainSubsystem drivetrain) { 
    this.timeToRunSeconds = timeToRunSeconds;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.targetAngle = targetAngle;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    this.timer.start();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
        ySpeed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
        Math.toRadians(drivetrain.getRotationOutput(targetAngle)),
        drivetrain.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeToRunSeconds);
  }
}
