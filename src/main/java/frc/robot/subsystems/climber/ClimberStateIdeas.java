// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

/** Add your docs here. */
public class ClimberStateIdeas {
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

  /**
   * The current bar the climber system is at
   */
  public enum CurrentBar {
    kMid(1), kHigh(2), kTraverse(3);

    private final int value;

    private CurrentBar(int value) {
      this.value = value;
    }

    public static CurrentBar fromValue(int value) {
      for (CurrentBar bar : CurrentBar.values()) {
        if (bar.value == value) {
          return bar;
        }
      }
      throw new IllegalArgumentException("No CurrentBar with a value of " + value + " exists");
    }
  }

   // Climb state management
   private ClimberStep climberStep = ClimberStep.kStep1;
   private CurrentBar currentBar = CurrentBar.kMid;

   public void setClimberState(ClimberStep state) {
    this.climberStep = state;
  }

  /**
   * Main method for performing climb steps
   */
  public void nextClimb() {
    int nextValue = this.climberStep.value + 1;
    if (nextValue > ClimberStep.values().length) {
      // we need to roll the climber target bar and roll over this climber step
    } else {
      this.climberStep = ClimberStep.fromValue(nextValue);
    }
  }

  private void nextBar() {
    int nextValue = this.currentBar.value + 1;
    if (nextValue > CurrentBar.values().length) {

    } else {
      // this.currentBar = CurrentBar.fromValue()
    }
  }

   
}
