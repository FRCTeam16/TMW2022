// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DMS;


import java.util.logging.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.ShooterFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LEDSubsystem extends SubsystemBase {
  private boolean running = false;
  private Timer timer = new Timer();
  private SerialPort serial;

  private static final double INITIAL_IGNORE_TIME = 1.0;

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
    buffer[1] = (byte) 0;
    buffer[2] = (byte) 0;
    buffer[3] = (byte) 0;
    buffer[4] = (byte) 0;
    buffer[5] = (byte) 0;
    buffer[6] = (byte) 0;
    buffer[7] = (byte) 0;
    buffer[8] = (byte) 0;
    buffer[9] = (byte) robotState;
    buffer[10] = (byte) allianceColor;
    buffer[11] = (byte) 0; //Subsystems.shooterSubsystem.targetRPM; //need to add a shooter up to speed dms for derrick
    buffer[12] = (byte) 0;
    buffer[13] = (byte) 0;
    buffer[14] = (byte) 255;

    this.serial.write(buffer, buffer.length);
  }

  public void begin() {
    timer.reset();
  }

  @Override
  public void periodic() {
    if (running) {
      
    }
  }

  private void runMotorTest() {

  }
}
