// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;

public class RunWithDisabledInstantCommand extends InstantCommand {
  /** Creates a new RunWithDisabledInstantCommand. */
  public RunWithDisabledInstantCommand(Runnable runnable) {
    super(runnable);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
