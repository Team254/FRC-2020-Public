package com.team254.frc2020.paths;

import com.team254.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    // TODO tune
    private static final double kMaxVel = 150.0;
    private static final double kMaxAccel = 100.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    public class TrajectorySet {
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectory;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectoryBack;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startToFarWOF1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> farWOF1ToShoot;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startingToPickup; // balls on bar by hangers
        public final Trajectory<TimedState<Pose2dWithCurvature>> pickupToTurningPoint;
        public final Trajectory<TimedState<Pose2dWithCurvature>> turningPointToFarWOF2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> farWOF2ToShootingPoint;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startingToNearWOF;
        public final Trajectory<TimedState<Pose2dWithCurvature>> nearWOFToShootingPoint1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> shootingPoint1ToShootingPoint2;

        public final Trajectory<TimedState<Pose2dWithCurvature>> turningPointToNearWOF;
        public final Trajectory<TimedState<Pose2dWithCurvature>> nearWOFToCloseShootingPoint;
        public final Trajectory<TimedState<Pose2dWithCurvature>> closeShootingPointToAlliancePickupPoint;

        private TrajectorySet() {
            testTrajectory = getTestTrajectory();
            testTrajectoryBack = getTestTrajectoryBack();

            startToFarWOF1 = getStartingToFarWOF();
            farWOF1ToShoot = getFarWOFToShoot();

            startingToPickup = getStartToPickup();
            pickupToTurningPoint = getPickupToTurningPoint();
            turningPointToFarWOF2 = getTurningPointToFarWOF2();
            farWOF2ToShootingPoint = getFarWOF2ToShoot();

            startingToNearWOF = getStartToNearWOF();
            nearWOFToShootingPoint1 = getNearWOFToShootingPoint1();
            shootingPoint1ToShootingPoint2 = getShootingPoint1ToShootingPoint2();

            turningPointToNearWOF = getTurningPointToNearWOF();
            nearWOFToCloseShootingPoint = getNearWOFToCloseShootingPoint();
            closeShootingPointToAlliancePickupPoint = getCloseShootingPointToAlliancePickupPoint();
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-120, 120, Rotation2d.fromDegrees(90)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectoryBack() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-120, 120, Rotation2d.fromDegrees(90)));
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartingToFarWOF() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-234, 0, Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(),
                    75, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarWOFToShoot() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-234, 0, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-90, 62, Rotation2d.fromDegrees(202.5)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToPickup() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-93.5, 65, Rotation2d.fromDegrees(116.59)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPickupToTurningPoint() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-93.5, 65, Rotation2d.fromDegrees(116.59)));
            waypoints.add(new Pose2d(-51, 4, Rotation2d.fromDegrees(180)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTurningPointToFarWOF2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-51, 4, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-240, 0, Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    95, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTurningPointToNearWOF() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-51, 4, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-180, 0, Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    95, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearWOFToCloseShootingPoint() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-180, 0, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-66, 62, Rotation2d.fromDegrees(180)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFarWOF2ToShoot() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-240, 0, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-90, 62, Rotation2d.fromDegrees(202.5)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getCloseShootingPointToAlliancePickupPoint() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-66, 62, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-150, 74, Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(),
                    75, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToNearWOF() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-90, 0, Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getNearWOFToShootingPoint1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-90, 0, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-12, -92, Rotation2d.fromDegrees(112.5)));
            return generateTrajectory(true, waypoints, Arrays.asList(),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getShootingPoint1ToShootingPoint2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-12, -92, Rotation2d.fromDegrees(112.5)));
            waypoints.add(new Pose2d(-60, -74, Rotation2d.fromDegrees(180)));
            Pose2d truss_entry_point = new Pose2d(-81, -92, Rotation2d.fromDegrees(-113));
            waypoints.add(truss_entry_point);
            waypoints.add(truss_entry_point.transformBy(new Pose2d(164, 0, Rotation2d.identity())));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    65, kMaxAccel, kMaxVoltage);
        }
    }
}