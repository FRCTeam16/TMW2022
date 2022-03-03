package frc.robot.subsystems.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PigeonGyro implements BSGyro {
    // private final Pigeon2 m_pigeon;
    private final WPI_Pigeon2 m_pigeon;
    private double[] ypr = new double[3];
    private double offset = 0.0;

    public PigeonGyro(int CAN_ID) {
        m_pigeon = new WPI_Pigeon2(CAN_ID);
        // var errorCode = m_pigeon.configFactoryDefault();
    }

    @Override
    public Rotation2d getGyroscopeRotation() {
        double degrees = m_pigeon.getYaw() % 360.0;
        degrees += offset;
        if (degrees < -180.0) {
            degrees += 360;
        } else if (degrees > 180.0) {
            degrees -= 360.0;
        }
        SmartDashboard.putNumber("PigeonGyro/Base", m_pigeon.getYaw());
        SmartDashboard.putNumber("PigeonGyro/Adjusted", degrees+offset);

        return Rotation2d.fromDegrees(degrees);
        // return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    @Override
    public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
        m_pigeon.setAccumZAngle(0);
    }

    @Override
    public void setGyroOffset(double offsetDegrees) {
        //m_pigeon.setYaw(offsetDegrees);
        this.offset = offsetDegrees;
        SmartDashboard.putNumber("PigeonGyro/Offset", offsetDegrees);
    }
}