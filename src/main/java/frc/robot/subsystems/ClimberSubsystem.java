// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  /*
   * Step 1 is the elevator is not extended
   * Step 2 is the solonoid firing forward
   * Step 3 is the elevator going up to fully hang on solonoid bars
   * Step 4 is the elevator extending
   * Step 5 is the solonoid reversing off of the previous bar
   */

  public enum ClimberStep {
    kStep1(1), kStep2(2), kStep3(3), kStep4(4), kStep5(5);

    private final int value;

    private ClimberStep(int value) {
      this.value = value;
    }

    public static ClimberStep fromValue(int value) {
      for (ClimberStep step : ClimberStep.values()) {
        if (step.value == value) {
          return step;
        }
      }
      throw new IllegalArgumentException("No ClimberStep with a value of " + value + " exists");
    }
  }

  public enum CurrentBar {
    kMid(1), kHigh(2), kTraverse(3);

    private final int value;

    private CurrentBar(int value) {
      this.value = value;
  }
}

  public static CurrentBar fromValue(int value) {
    for (CurrentBar bar : CurrentBar.values()) {
      if (bar.value == value) {
        return bar;
      }
    }
    throw new IllegalArgumentException("No CurrentBar with a value of " + value + " exists");
  }

  private final CANSparkMax climberMotor = new CANSparkMax(Constants.RIGHTCLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax followerMotor = new CANSparkMax(Constants.LEFTCLIMBER_MOTOR_ID, MotorType.kBrushless);
  private final DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 6);

  private ClimberStep climberStep = ClimberStep.kStep1;
  private CurrentBar currentBar = CurrentBar.kMid;

  private DoubleSolenoid.Value currentSolenoidValue = DoubleSolenoid.Value.kOff;

  private final double DEFAULT_ELEVDOWN_SPEED = -0.35;
  private static final String ELEVDOWN_SPEED_KEY = "Climber Speed";

  private final double DEFAULT_ELEVUP_SPEED = 0.20;
  private static final String ELEVUP_SPEED_KEY = "Extend Speed";

  private final double DEFAULT_ELEVHOLD_SPEED = -0.15;
  private static final String ELEVHOLD_SPEED_KEY = "Hold Speed";

  private final double DEFAULT_DISABLED_SPEED = 0;
  private static final String DISABLED_SPEED_KEY = "DISABLED Speed";

  public ClimberSubsystem() {

    followerMotor.follow(climberMotor, true);

    SmartDashboard.setDefaultNumber(ELEVDOWN_SPEED_KEY, DEFAULT_ELEVDOWN_SPEED);
    SmartDashboard.setDefaultNumber(ELEVUP_SPEED_KEY, DEFAULT_ELEVUP_SPEED);
    SmartDashboard.setDefaultNumber(ELEVHOLD_SPEED_KEY, DEFAULT_ELEVHOLD_SPEED);
    SmartDashboard.setDefaultNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);

    CommandScheduler.getInstance().schedule(new CommandBase() {
      @Override
      public void execute() {
        // SmartDashboard.putNumber("Climber Encder",
        // climberMotor.getSelectedSensorPosition());
        SmartDashboard.putString("Climber State", climberStep.name());

      }

      @Override
      public boolean runsWhenDisabled() {
        return true;

      }
    });

  }

  public void setClimberState(ClimberStep state) {

    this.climberStep = state;

  }

  public void SolonoidFWD() {
    currentSolenoidValue = Value.kForward;
   
  }

  public void SolonoidREV() {
    currentSolenoidValue = Value.kReverse;
  }

  public void nextClimb() {
    int nextValue = this.climberStep.value + 1;
    if (nextValue > ClimberStep.values().length) {
      // we need to roll the climber target bar and roll over this climber step
    } else {
      this.climberStep = ClimberStep.fromValue(nextValue);
    }
  }

  public void nextBar() {
    int nextValue = this.currentBar.value + 1;
    if(nextValue > CurrentBar.values().length) {

    } else {
    //this.currentBar = CurrentBar.fromValue() 
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pullSpeed = SmartDashboard.getNumber(ELEVDOWN_SPEED_KEY, DEFAULT_ELEVDOWN_SPEED);
    double releaseSpeed = SmartDashboard.getNumber(ELEVHOLD_SPEED_KEY, DEFAULT_ELEVHOLD_SPEED);
    double extendSpeed = SmartDashboard.getNumber(ELEVUP_SPEED_KEY, DEFAULT_ELEVUP_SPEED);
    double defaultSpeed = SmartDashboard.getNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);
    //double solenoidFWD = 
    //solenoidREV = SolonoidREV();

    //climberSolenoid.set(currentSolenoidValue);

    final double climberOutput;

    switch (climberStep) {
      case kStep1:
        climberOutput = pullSpeed;
        break;

      case kStep2:
        //climberOutput = solenoidFWD;
        break;

      case kStep3:
        climberOutput = releaseSpeed;
        break;

      case kStep4:
        climberOutput = extendSpeed;
        break;

      case kStep5:
        //climberOutput = solenoidREV;
        break;

      default:
        climberOutput = 0.0;

    }

    //climberMotor.set(climberOutput);
  }
}
