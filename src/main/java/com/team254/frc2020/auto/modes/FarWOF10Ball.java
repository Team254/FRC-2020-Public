package com.team254.frc2020.auto.modes;

import com.team254.frc2020.Constants;
import com.team254.frc2020.auto.AutoModeEndedException;
import com.team254.frc2020.auto.actions.*;
import com.team254.frc2020.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Rotation2d;

import java.util.List;

// zero -> steel bars by rendezvous point -> back to zeroish -> far wof -> shooting point
public class FarWOF10Ball extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DeployIntakeAction(true));
        runAction(new RunIntakeAction());
        runAction(new ParallelAction(
                new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().startingToPickup),
                new AutoAimAction(Rotation2d.fromDegrees(30))));


        runAction(
                new ParallelAction(
                        new ShootAction(Constants.kCoarseShootingParams, 2),
                        new SeriesAction(
                                new WaitAction(0.5),
                                new StopIntakingAction(),
                                new DeployIntakeAction(false)
                        )));
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().pickupToTurningPoint));

        runAction(new DeployIntakeAction(true));
        runAction(new RunIntakeAction());

        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().turningPointToFarWOF2));

        runAction(new ParallelAction(List.of(
                new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().farWOF2ToShootingPoint),
                new AutoAimAction(Rotation2d.fromDegrees(0)),
                new SeriesAction(
                        new WaitAction(1.0),
                        new StopIntakingAction(),
                        new DeployIntakeAction(false)
                )
        )
        ));
        runAction(new ShootAction(Constants.kCoarseShootingParams, 2));
    }
}