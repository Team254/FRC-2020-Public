package com.team254.lib.vision;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class AimingParameters {
    private final double range;
    private final Pose2d field_to_vehicle;
    private final Pose2d field_to_goal;
    private final Rotation2d robot_to_goal_rotation;
    private final double last_seen_timestamp;
    private final double stability;
    private final int track_id;

    public AimingParameters(Pose2d field_to_vehicle,
                            Pose2d field_to_goal, double last_seen_timestamp,
                            double stability, int track_id) {
        this.field_to_vehicle = field_to_vehicle;
        this.field_to_goal = field_to_goal;
        final Pose2d vehicle_to_goal = field_to_vehicle.inverse().transformBy(field_to_goal);
        this.range = vehicle_to_goal.getTranslation().norm();
        this.robot_to_goal_rotation = vehicle_to_goal.getTranslation().direction();
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
        this.track_id = track_id;
    }

    public Pose2d getFieldToVehicle() {
        return field_to_vehicle;
    }

    public Pose2d getFieldToGoal() {
        return field_to_goal;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getRobotToGoalRotation() {
        return robot_to_goal_rotation;
    }

    public double getLastSeenTimestamp() {
        return last_seen_timestamp;
    }

    public double getStability() {
        return stability;
    }

    public int getTrackId() {
        return track_id;
    }
}