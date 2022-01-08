package com.team254.frc2020;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

/**
 * Ensure we don't push bad constants
 */
@RunWith(JUnit4.class)
public class ConstantsTest {
    @Test
    public void test() {
        Assert.assertFalse("Still in Gamepad Drive Mode", Constants.kUseDriveGamepad);
        Assert.assertFalse("Still in Hood Tuning Mode", Constants.kIsHoodTuning);
    }
}