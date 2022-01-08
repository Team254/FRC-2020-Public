package com.team254.frc2020.auto.actions;

import com.team254.frc2020.subsystems.Drive;

public class OverrideTrajectoryAction extends RunOnceAction {
    @Override
    public void runOnce() {
        Drive.getInstance().overrideTrajectory(true);
    }
}