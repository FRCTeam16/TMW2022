package frc.robot.commands.testing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;

public class ProfiledDistanceDriveCommand extends CommandBase {

  public static boolean debug = false;  // controls writing sysouts

  private double distanceThreshold = 0.05;
  private double angle;
  private double maxSpeed;
  private double xdist;
  private double ydist;

  private Translation2d startPose;
  private Translation2d targetPose;

  private Constraints constraints;
  private State goal;
  private State currentState;
  private double endSpeed = 0.0;
  private boolean fieldCentric = true;


  /**
   * Uses trapezoid profile to drive a distance
   * @param angleDegrees
   * @param speed - percent speed
   * @param xMeters
   * @param yMeters
   */
  public ProfiledDistanceDriveCommand(double angleDegrees, double speed, double xMeters, double yMeters) {
    this.angle = angleDegrees;
    this.maxSpeed = MathUtil.clamp(speed, -1, 1) * Constants.Auto.MaxSpeedMetersPerSecond;
    this.xdist = xMeters;
    this.ydist = yMeters;

    this.constraints = new TrapezoidProfile.Constraints(
      maxSpeed,
      Constants.Auto.MaxAccelerationMetersPerSecondSquared
    );

    addRequirements(Subsystems.drivetrainSubsystem);
  }

  /**
   * Overrides the default threshold for determining we are close enough
   * @param threshold
   * @return
   */
  public ProfiledDistanceDriveCommand withThreshold(double threshold) {
    this.distanceThreshold = threshold;
    return this;
  }

  public ProfiledDistanceDriveCommand withEndSpeed(double percentSpeed) {
    this.endSpeed = MathUtil.clamp(percentSpeed, -1, 1) * Constants.Auto.MaxSpeedMetersPerSecond;
    return this;
  }

  public ProfiledDistanceDriveCommand withRobotCentric() {
    this.fieldCentric = false;
    return this;
  }

  public ProfiledDistanceDriveCommand withConstraints(TrapezoidProfile.Constraints constraints) {
    this.constraints = constraints;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startPose = Subsystems.drivetrainSubsystem.getPose().getTranslation();
    var translation = new Translation2d(xdist, ydist);
    if (!fieldCentric) {
      translation = translation.rotateBy(Subsystems.drivetrainSubsystem.getGyroscopeRotation());
    }
    var xy_trans = startPose.plus(translation);
    this.targetPose = new Translation2d(xy_trans.getX(), xy_trans.getY());

    this.currentState = new TrapezoidProfile.State(0, 0); // assume zero initial velocity
    this.goal = new TrapezoidProfile.State(startPose.getDistance(xy_trans), endSpeed);

    System.out.println("****************> SDDC Initialize");
    System.out.println("Current Pose: " + startPose);
    System.out.println("Target Pose : " + targetPose);
    System.out.println("<**************** SDDC Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentPose = Subsystems.drivetrainSubsystem.getPose().getTranslation();

    // Use velocity from last state, but update out position
    this.currentState = new TrapezoidProfile.State(currentPose.getDistance(startPose), this.currentState.velocity);
    var profile = new TrapezoidProfile(constraints, goal, currentState);

    // Calculate what our next position and velocity should be
    this.currentState = profile.calculate(0.2);
    
    var speed = currentState.velocity;

    //
    var driveAngle = Math.atan2(
      targetPose.getY() - currentPose.getY(),
      targetPose.getX() - currentPose.getX());

    double vxMetersPerSecond = Math.abs(speed * Math.cos(driveAngle));
    double vyMetersPerSecond = Math.abs(speed * Math.sin(driveAngle));
    
    
    // TODO: may just need to add 90 to driveAngle and use true quadrant sign values
    if (currentPose.getY() > targetPose.getY()) {
      vyMetersPerSecond = -vyMetersPerSecond;
    }
    if (currentPose.getX() > targetPose.getX()) {
      vxMetersPerSecond = -vxMetersPerSecond;
    }
/* 
Test new
    var driveAngle = Math.atan2(
      targetPose.getX() - currentPose.getX()
      targetPose.getY() - currentPose.getY());

    double vxMetersPerSecond = speed * Math.cos(driveAngle);
    double vyMetersPerSecond = speed * Math.sin(driveAngle);
*/

    if (debug) {
    System.out.println(
      "DA: " + driveAngle +
      " | VX: " + vxMetersPerSecond / Constants.Auto.MaxSpeedMetersPerSecond + 
      " | VY: " + vyMetersPerSecond / Constants.Auto.MaxSpeedMetersPerSecond);
    }

    var twist = Subsystems.drivetrainSubsystem.getRotationController().calculate(
        Subsystems.drivetrainSubsystem.getGyroscopeRotation().getDegrees(), angle);
    var speeds = (fieldCentric) ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(
          vxMetersPerSecond, 
          vyMetersPerSecond,
          Math.toRadians(twist), 
          Subsystems.drivetrainSubsystem.getGyroscopeRotation()) :
        new ChassisSpeeds(
          vxMetersPerSecond,
          vyMetersPerSecond,
          Math.toRadians(twist)
        );
    
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
    if (debug) {
      System.out.println("DIST: " + distance + " | T:" + targetPose + " | C:" + Subsystems.drivetrainSubsystem.getPose().getTranslation());
    }
    return (distance < distanceThreshold);
  }
}
