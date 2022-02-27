package frc.robot.subsystems.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonGyro implements BSGyro {
    // private final Pigeon2 m_pigeon;
    private final WPI_Pigeon2 m_pigeon;
    private double[] ypr = new double[3];

    public PigeonGyro(int CAN_ID) {
        m_pigeon = new WPI_Pigeon2(CAN_ID);
        // var errorCode = m_pigeon.configFactoryDefault();
    }

    @Override
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    @Override
    public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
        m_pigeon.setAccumZAngle(0);
    }

    @Override
    public void setGyroOffset(double offsetDegrees) {
        m_pigeon.setYaw(offsetDegrees);
        
    }
}