// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;

public class ShooterFeederSubsystem extends SubsystemBase {
  private boolean enabled = false;

  // Calling CANSpark motor 

   private final CANSparkMax feederMotor = new CANSparkMax(Constants.SHOOTERFEEDER_MOTOR_ID, MotorType.kBrushless);
   private final double DEFAULT_FEEDER_SPEED = -.4;
   private static final String FEEDER_SPEED_KEY = "Feeder Speed";

  /** Creates a new ShooterFeederSubsystem. */
  public ShooterFeederSubsystem() {
    feederMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    SmartDashboard.setDefaultNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
  }
  public void pull() {
    if(Subsystems.shooterSubsystem.atMinimumSpeed()) {
    this.enabled = true;
    }
  }

  public void dontPull() {
    this.enabled = false;
  
  } 

  @Override
  public void periodic() {
    double feederSpeed = 0.0;
    
   
    if (enabled) {
      feederSpeed = SmartDashboard.getNumber(FEEDER_SPEED_KEY, DEFAULT_FEEDER_SPEED);
      
      
    }
    
    feederMotor.set(feederSpeed);
    
    
  
}
}
