package frc.robot.subsystems.gyro;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class PigeonGyro implements BSGyro {
    private final Pigeon2 m_pigeon;

    public PigeonGyro(int CAN_ID) {
        m_pigeon = new Pigeon2(CAN_ID);
    }

    @Override
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    @Override
    public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
    }


// public PigeonGyro(int CAN_ID) {
//     m_pigeon = new PigeonIMU(CAN_ID);
// }

// @Override
// public Rotation2d getGyroscopeRotation() {
//     return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
// }

// @Override
// public void zeroGyroscope() {
//     m_pigeon.setFusedHeading(0.0);
// }

}