// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class RotationController extends PIDController {
    
    private static final double kP = 3.5;
    private static final double kI = 0.0; // 3.0;
    private static final double kD = 0.0;

    private double tolerance = 1.0;

    public RotationController() {
        this(kP, kI, kD);
    }

    public RotationController(double kp, double ki, double kd) {
        super(kp, ki, kd);
        this.enableContinuousInput(-180, 180);
        this.setTolerance(tolerance);
    }
}
