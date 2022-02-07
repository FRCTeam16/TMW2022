// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Arrays;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import edu.wpi.first.hal.HAL;

/** Add your docs here. */
public class TestBSPrefs {

    @Before
    public void setUp() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    }

    @Test
    public void testOffsetsExist() {
        var prefs = new BSPrefs("src/main/deploy/bsprefs.csv");
        Arrays.asList("FLOff", "FROff", "RLOff", "RROff").forEach(key -> {
            var offset = prefs.getDouble("FLOff", -999);
            Assert.assertNotEquals(-999, offset);
        });
    }
}
