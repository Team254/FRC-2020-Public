package com.team254.frc2020.auto.modes;

import com.team254.frc2020.auto.AutoModeEndedException;

public class DoNothingAutoMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Do nothing auto mode");
    }
}