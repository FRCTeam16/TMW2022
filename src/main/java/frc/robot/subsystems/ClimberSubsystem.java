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
import com.revrobotics.SparkMaxPIDController;

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

  private static final int deviceID = 1;
  private CANSparkMax climberMotor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  

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

    m_pidController = climberMotor.getPIDController();
    m_encoder = climberMotor.getEncoder();

    SmartDashboard.setDefaultNumber(ELEVDOWN_SPEED_KEY, DEFAULT_ELEVDOWN_SPEED);
    SmartDashboard.setDefaultNumber(ELEVUP_SPEED_KEY, DEFAULT_ELEVUP_SPEED);
    SmartDashboard.setDefaultNumber(ELEVHOLD_SPEED_KEY, DEFAULT_ELEVHOLD_SPEED);
    SmartDashboard.setDefaultNumber(DISABLED_SPEED_KEY, DEFAULT_DISABLED_SPEED);

    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    //limits the Velocity in RPM of the PID
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);

    //lower bound in RPM
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);

    //limit the acceleration in RPM squared of the PID
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

    //set the max allowed error for the PID
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);



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

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      processVariable = m_encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      processVariable = m_encoder.getPosition();
    }
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", m_motor.getAppliedOutput());

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
