// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
  private SerialPort arduino;
  private Timer timer;
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
  // public void updateLEDbuffer() {
  //   LEDbuffer[0] = 254;       //  initialize
  //   LEDbuffer[1] = FLD;
  //   LEDbuffer[2] = FLS;
  //   LEDbuffer[3] = FRD;
  //   LEDbuffer[4] = FRS;
  //   LEDbuffer[5] = RLD;
  //   LEDbuffer[6] = RLS;
  //   LEDbuffer[7] = RRD;
  //   LEDbuffer[8] = RRS;
  //   LEDbuffer[9] = commStatus;
  //   LEDbuffer[10] = allianceColor;
  //   LEDbuffer[11] = a;
  //   LEDbuffer[12] = b;
  //   LEDbuffer[13] = c;
  //   LEDbuffer[14] = 255;     //  terminate
  // }

  }
  @Override
  

  public void periodic() {
    // This method will be called once per scheduler run
  }
}
