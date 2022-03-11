// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.69; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.52; // FIXME Measure and set wheelbase



    public static final int INTAKE_MOTOR_ID = 15;
    //public static final int BEATER_MOTOR_ID = 10;
    //public static final int BEATER2?_MOTOR_ID = X;


        //Climber motors 
    public static final int RIGHTCLIMBER_MOTOR_ID = 16;
    public static final int LEFTCLIMBER_MOTOR_ID = 13;

    public static final int SHOOTERWHEELRIGHT_MOTOR_ID = 14; // Motor ID for One of the shooter motors
    public static final int SHOOTERWHEELLEFT_MOTOR_ID = 17; // Motor ID for one of the shooter motors
    public static final int SHOOTERFEEDER_MOTOR_ID = 11; // Motor ID for the feeder for the shooter
    public static final int TURRET_MOTOR_ID = 18; //Motor ID for the turret
    
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(216.47); // Practice Robo = 351.7

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(24.0); // Practice Robo = 36.8

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(287.49); // 295 28.64

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(225.96); // Practice Robo = 347.6


    public static final int PIGEON_ID = 00;

    public static final class Auto {
        public static final double MaxSpeedMetersPerSecond = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * .75;
        public static final double MaxAccelerationMetersPerSecondSquared = DrivetrainSubsystem.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;

        public static final double Px = 1;
        public static final double Py = Px;
        public static final double ThetaP = Math.toRadians(1); 

        public static final TrapezoidProfile.Constraints ThetaControllerConstraints = new TrapezoidProfile.Constraints(
                DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                DrivetrainSubsystem.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

    }
}
