package com.team254.frc2020.auto.modes;

import com.team254.frc2020.Constants;
import com.team254.frc2020.auto.AutoModeEndedException;
import com.team254.frc2020.auto.actions.*;
import com.team254.frc2020.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Rotation2d;

// zero -> steel bars by rendezvous point -> back to zeroish -> far wof -> shooting point
public class NearWOF10Ball extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DeployIntakeAction(true));
        runAction(new RunIntakeAction());
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().startingToNearWOF));
        runAction(new StopIntakingAction());
        runAction(new DeployIntakeAction(false));

        runAction(new ParallelAction(
                new AutoAimAction(Rotation2d.fromDegrees(-30)),
                new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().nearWOFToShootingPoint1)
        ));
        runAction(new ShootAction(Constants.kCoarseShootingParams, 2.0));

        runAction(new DeployIntakeAction(true));
        runAction(new RunIntakeAction());
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().shootingPoint1ToShootingPoint2));
        runAction(new AutoAimAction(Rotation2d.fromDegrees(0)));
        runAction(new StopIntakingAction());
        runAction(new DeployIntakeAction(false));
        runAction(new ShootAction(Constants.kCoarseShootingParams, 3.0));
    }
}