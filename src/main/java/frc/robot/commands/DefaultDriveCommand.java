package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_fieldRelativeSupplier;
    private boolean shootThresholdSpeedEnabled = false;

    

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier fieldRelativeSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_fieldRelativeSupplier = fieldRelativeSupplier;
        SmartDashboard.setDefaultNumber("Clamped Shooting Drive Velocity", 2*DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement

        double translationX = m_translationXSupplier.getAsDouble();
        double translationY = m_translationYSupplier.getAsDouble();
        double rotationSupplier = m_rotationSupplier.getAsDouble();
        Rotation2d gyroRotation = m_drivetrainSubsystem.getGyroscopeRotation();

        if (shootThresholdSpeedEnabled && Subsystems.feederSubsystem.isShooting()) {
             double maxSpeed = SmartDashboard.getNumber("Clamped Shooting Drive Velocity", .2*DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
             double translation = Math.sqrt(translationX*translationX+translationY*translationY);

             translation = MathUtil.clamp(translation, -maxSpeed, maxSpeed);
             
             double angle = Math.atan2(translationY,translationX);

             translationX = translation*Math.cos(angle)*Math.signum(translationX);
             translationY = translation*Math.sin(angle)*Math.signum(translationY);
        }

        final ChassisSpeeds chassisSpeeds = (m_fieldRelativeSupplier.getAsBoolean()) ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationX,
                translationY,
                rotationSupplier,
                gyroRotation
                ) :
            new ChassisSpeeds(
                translationX,
                translationY,
                rotationSupplier);

        m_drivetrainSubsystem.drive(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    
}
