// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();


  private final XboxController gamepad = new XboxController(2);
  private final Joystick leftJoy = new Joystick(0);
  private final Joystick rightJoy = new Joystick(1);

  private static final SlewRateLimiter xRateLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter yRateLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(.7);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
   
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
    //     () -> -modifyAxis(xRateLimiter.calculate(gamepad.getY(Hand.kLeft))) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(yRateLimiter.calculate(gamepad.getX(Hand.kLeft))) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis(rotRateLimiter.calculate(gamepad.getX(Hand.kRight)))
    //         * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_drivetrainSubsystem.setDefaultCommand(
      new DefaultDriveCommand(m_drivetrainSubsystem,
        () -> -OIUtil.modifyAxis(xRateLimiter.calculate(rightJoy.getY())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -OIUtil.modifyAxis(yRateLimiter.calculate(rightJoy.getX())) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -OIUtil.modifyAxis(rotRateLimiter.calculate(leftJoy.getX())) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> !leftJoy.getRawButton(13))
    );

    // Configure the button bindings
    configureButtonBindings();

    //Zero Out the Gyroscope
    m_drivetrainSubsystem.zeroGyroscope();

    // Debug telemetry
    CommandScheduler.getInstance().schedule(new CommandBase()  {
      @Override
      public void execute() {
        
        SmartDashboard.putNumber("LX", rightJoy.getX());
        SmartDashboard.putNumber("LY", rightJoy.getY());

        SmartDashboard.putNumber("GX", gamepad.getX(Hand.kLeft));
        SmartDashboard.putNumber("GY", gamepad.getY(Hand.kLeft));
        

        SmartDashboard.putNumber("Gyro", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());
      }

      @Override
        public boolean runsWhenDisabled() {
          return true;
        }

    });
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    new Button(gamepad::getBackButton)
        // No requirements because we don't need to interrupt anything
        .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    new Button(rightJoy::getTrigger)
      .whenPressed(m_intakeSubsystem::enable)
      .whenReleased(m_intakeSubsystem::disable);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

}
