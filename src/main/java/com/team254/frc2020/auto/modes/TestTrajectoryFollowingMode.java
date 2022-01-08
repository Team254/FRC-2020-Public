package com.team254.frc2020.auto.modes;

import com.team254.frc2020.auto.AutoModeEndedException;
import com.team254.frc2020.auto.actions.DriveTrajectoryAction;
import com.team254.frc2020.paths.TrajectoryGenerator;

public class TestTrajectoryFollowingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectory));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().testTrajectoryBack));
    }
}