package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ClimberClosedLoopManager {
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private double kP = 0.1;
    private double kI = 0; 
    private double kD = 0;
    private double kIz = 0;
    private double kFF = 0;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;

    private TrapezoidProfile.Constraints contraints = new TrapezoidProfile.Constraints(10, 5);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private boolean configMode = false;


    public ClimberClosedLoopManager(CANSparkMax climberMotor) {
        this.m_pidController = climberMotor.getPIDController();
        this.m_encoder = climberMotor.getEncoder();

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);

        if (configMode) {
          SmartDashboard.putNumber("Climber/Closed/P Gain", kP);
          SmartDashboard.putNumber("Climber/Closed/I Gain", kI);
          SmartDashboard.putNumber("Climber/Closed/D Gain", kD);
          SmartDashboard.putNumber("Climber/Closed/I Zone", kIz);
          SmartDashboard.putNumber("Climber/Closed/Feed Forward", kFF);
          SmartDashboard.putNumber("Climber/Closed/Max Output", kMaxOutput);
          SmartDashboard.putNumber("Climber/Closed/Min Output", kMinOutput);
        }
    }


    public void setTarget(double target) {
      this.setpoint = new TrapezoidProfile.State(this.m_encoder.getPosition(), 0);
      this.goal = new TrapezoidProfile.State(target, 0);
      SmartDashboard.putNumber("Climber/Closed/Target", target);
    }

    public void run() {
      if (configMode) {
        double p = SmartDashboard.getNumber("Climber/Closed/P Gain", 0);
        double i = SmartDashboard.getNumber("Climber/Closed/I Gain", 0);
        double d = SmartDashboard.getNumber("Climber/Closed/D Gain", 0);
        double iz = SmartDashboard.getNumber("Climber/Closed/I Zone", 0);
        double ff = SmartDashboard.getNumber("Climber/Closed/Feed Forward", 0);
        double max = SmartDashboard.getNumber("Climber/Closed/Max Output", 0);
        double min = SmartDashboard.getNumber("Climber/Closed/Min Output", 0);

        if ((p != kP)) {
            m_pidController.setP(p);
            kP = p;
          }
          if ((i != kI)) {
            m_pidController.setI(i);
            kI = i;
          }
          if ((d != kD)) {
            m_pidController.setD(d);
            kD = d;
          }
          if ((iz != kIz)) {
            m_pidController.setIZone(iz);
            kIz = iz;
          }
          if ((ff != kFF)) {
            m_pidController.setFF(ff);
            kFF = ff;
          }
          if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
          }
        }
        m_pidController.setP(kP);
        var profile = new TrapezoidProfile(contraints, goal, setpoint);
        setpoint = profile.calculate(0.2);  // look one scan ahead

        m_pidController.setReference(setpoint.position, CANSparkMax.ControlType.kPosition);
    }

}
