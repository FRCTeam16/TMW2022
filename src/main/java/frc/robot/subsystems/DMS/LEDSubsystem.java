// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DMS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;

public class LEDSubsystem extends SubsystemBase {
    private boolean running = false;
    private Timer timer = new Timer();
    private SerialPort serial;

    private enum DMSPhase {
        Stopped, RunDriveMotors, RunSteerMotors
    }
    private DMSPhase currentPhase = DMSPhase.Stopped;
    private DMSStats driveDmsStatus = new DMSStats();
    private DMSStats steerDmsStatus = new DMSStats();
    private DriveInfo<Integer> driveStatus = new DriveInfo<Integer>(0);
    private DriveInfo<Integer> steerStatus = new DriveInfo<Integer>(0);

    private static final double INITIAL_IGNORE_TIME = 1.0;
    private static final double MOTOR_TEST_TIME = 4.0;


    /** Creates a new LEDSubsystem. */
    public LEDSubsystem() {
        // public void updateLEDbuffer() {
        // LEDbuffer[0] = 254; // initialize
        // LEDbuffer[1] = FLD;
        // LEDbuffer[2] = FLS;
        // LEDbuffer[3] = FRD;
        // LEDbuffer[4] = FRS;
        // LEDbuffer[5] = RLD;
        // LEDbuffer[6] = RLS;
        // LEDbuffer[7] = RRD;
        // LEDbuffer[8] = RRS;
        // LEDbuffer[9] = commStatus;
        // LEDbuffer[10] = allianceColor;
        // LEDbuffer[11] = a;
        // LEDbuffer[12] = b;
        // LEDbuffer[13] = c;
        // LEDbuffer[14] = 255; // terminate
        // }

        try {
            serial = new SerialPort(115200, SerialPort.Port.kUSB1);
        } catch (Exception e) {
            System.err.println("Unable to create DMS/LED subsystem, problem with serial port: " + e.getMessage());
            // TODO: probably set bool preventing running
        }
    }

    public void Report() {
        if (running && serial != null) {
            SendData(new DriveInfo<Double>(0.0), new DriveInfo<Double>(0.0));
        }
    }

    public void SendData(DriveInfo<Double> driveMotor, DriveInfo<Double> steerMotor) {
        int robotState = 0;
        if (DriverStation.isDisabled()) {
            robotState = 1;
        } else if (DriverStation.isAutonomous()) {
            robotState = 2;
        } else if (DriverStation.isTeleop()) {
            robotState = 3;
        }

        int allianceColor = 0;
        if (DriverStation.getAlliance() == Alliance.Red) {
            allianceColor = 1;
        } else if (DriverStation.getAlliance() == Alliance.Blue) {
            allianceColor = 2;
        }
        byte[] buffer = new byte[15];

        buffer[0] = (byte) 254;
        buffer[1] = driveStatus.FL.byteValue();
        buffer[2] = driveStatus.FR.byteValue();
        buffer[3] = driveStatus.RL.byteValue();
        buffer[4] = driveStatus.RR.byteValue();
        buffer[5] = steerStatus.FL.byteValue();
        buffer[6] = steerStatus.FL.byteValue();
        buffer[7] = steerStatus.FL.byteValue();
        buffer[8] = steerStatus.FL.byteValue();
        buffer[9] = (byte) robotState;
        buffer[10] = (byte) allianceColor;
        buffer[11] = (byte) (Subsystems.shooterSubsystem.atMinimumSpeed() ? 1 : 0); // Subsystems.shooterSubsystem.targetRPM; //need to add a shooter up to speed
                               // dms for derrick
        buffer[12] = (byte) (Subsystems.turretSubsystem.hasVisionTarget() ? 1 : 0);
        buffer[13] = (byte) 0;
        buffer[14] = (byte) 255;

        this.serial.write(buffer, buffer.length);
    }

    public void begin() {
        timer.reset();
    }

    public void stopSubsystem() {
        running = false;
    }

    public void startDMS() {
        timer.reset();
        timer.start();
        driveDmsStatus = new DMSStats();
        steerDmsStatus = new DMSStats();

        driveStatus = new DriveInfo<Integer>(0);
        steerStatus = new DriveInfo<Integer>(0);

        currentPhase = DMSPhase.RunDriveMotors;
    }

    public boolean isStopped() {
        return currentPhase == DMSPhase.Stopped;
    }
    
    public void stopDMS() {
        currentPhase = DMSPhase.Stopped;
        
        driveStatus = new DriveInfo<Integer>(0);
        steerStatus = new DriveInfo<Integer>(0);
    }

    @Override
    public void periodic() {
        if (running) {
            switch (currentPhase) {
                case Stopped:
                    break;
                case RunDriveMotors:
                    runMotorTest();
                    break;
                case RunSteerMotors:
                    runSteerTest();
                    break;
            }
            ;
        }
    }

    private void runMotorTest() {
        final double now = timer.get();
        if (now < MOTOR_TEST_TIME) {
            Subsystems.drivetrainSubsystem.DMSDrive(1.0);
            Subsystems.drivetrainSubsystem.DMSSteer(0.0);

            if (now > INITIAL_IGNORE_TIME) {
                driveDmsStatus.addDriveCurrent(Subsystems.drivetrainSubsystem.getDriveOutputCurrent());
                driveDmsStatus.addDriveVelocity(Subsystems.drivetrainSubsystem.getDriveVelocity());

                DMSStats.print("(DVel)", driveDmsStatus.velocity);
                DMSStats.print("(DAmp)", driveDmsStatus.current);

                double velAvg = DMSStats.average(driveDmsStatus.velocity);
                double ampAvg = DMSStats.average(driveDmsStatus.current);
                System.out.println("Vel Avg: " + velAvg + " | Amp Avg: " + ampAvg);
                
                driveStatus = driveDmsStatus.calculateStatus();
                DMSStats.print("[Drive Status]", driveStatus);
            } else {
                currentPhase = DMSPhase.RunSteerMotors;
                timer.reset();
            }
        }
    }

    private void runSteerTest() {
        final double now = timer.get();
        if (now < MOTOR_TEST_TIME) {
            Subsystems.drivetrainSubsystem.DMSDrive(0.0);
            Subsystems.drivetrainSubsystem.DMSSteer(1.0);

            if (now > INITIAL_IGNORE_TIME) {
                steerDmsStatus.addDriveCurrent(Subsystems.drivetrainSubsystem.getSteerOutputCurrent());
                steerDmsStatus.addDriveVelocity(Subsystems.drivetrainSubsystem.getSteerVelocity());

                DMSStats.print("(SVel)", steerDmsStatus.velocity);
                DMSStats.print("(SAmp)", steerDmsStatus.current);

                double velAvg = DMSStats.average(steerDmsStatus.velocity);
                double ampAvg = DMSStats.average(steerDmsStatus.current);
                System.out.println("Vel Avg: " + velAvg + " | Amp Avg: " + ampAvg);
                
                steerStatus = steerDmsStatus.calculateStatus();
                DMSStats.print("[Steer Status]", driveStatus);
            } else {
                currentPhase = DMSPhase.Stopped;
            }
        }
    }

}
