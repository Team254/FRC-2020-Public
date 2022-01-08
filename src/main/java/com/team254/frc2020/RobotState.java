package com.team254.frc2020;

import com.team254.frc2020.subsystems.Limelight;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 100;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Turret frame: origin is the center of the turret.
     *
     * 4. Camera frame: origin is the center of the Limelight relative to the
     * turret.
     *
     * 5. Target frame: origin is the center of the vision target, facing outwards
     * along the normal.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Vehicle-to-turret: Rotation measured by the turret encoder; translation is constant.
     *
     * 3. Turret-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-target: Measured by the vision system.
     */

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> vehicle_to_turret_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;

    private GoalTracker goal_tracker_ = new GoalTracker();

    private RobotState() {
        reset(0.0, Pose2d.identity(), Pose2d.identity());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle,
                                   Pose2d initial_vehicle_to_turret) {
        reset(start_time, initial_field_to_vehicle);
        vehicle_to_turret_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        vehicle_to_turret_.put(new InterpolatingDouble(start_time), initial_vehicle_to_turret);
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity(), Pose2d.identity());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getVehicleToTurret(double timestamp) {
        return vehicle_to_turret_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestVehicleToTurret() {
        return vehicle_to_turret_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addVehicleToTurretObservation(double timestamp, Pose2d observation) {
        vehicle_to_turret_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Twist2d displacement, Twist2d measured_velocity,
                                             Twist2d predicted_velocity) {
        distance_driven_ += displacement.dx;
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), displacement));
        vehicle_velocity_measured_ = measured_velocity;
        if (Math.abs(vehicle_velocity_measured_.dtheta) < 2.0 * Math.PI) {
            // Reject really high angular velocities from the filter.
            vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        } else {
            vehicle_velocity_measured_filtered_.add(new Twist2d(vehicle_velocity_measured_.dx, vehicle_velocity_measured_.dy, 0.0));
        }
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized void resetVision() {
        goal_tracker_.reset();
    }

    private Translation2d getCameraToVisionTargetTranslation(TargetInfo target, Limelight source) {
        return Constants.kUseTopCorners ?
                getCameraToTopVisionTargetTranslation(target, source.getLensHeight(), source.getHorizontalPlaneToLens()) :
                getCameraToBottomVisionTargetTranslation(target, source.getLensHeight(), source.getHorizontalPlaneToLens());
    }

    public static Translation2d getCameraToTopVisionTargetTranslation(TargetInfo target, double cameraHeight, Rotation2d cameraPitch) {
        return getCameraToVisionTargetTranslation(target, cameraHeight, cameraPitch, Constants.kTopVisionTargetHeight);
    }

    private Translation2d getCameraToBottomVisionTargetTranslation(TargetInfo target, double cameraHeight, Rotation2d cameraPitch) {
        return getCameraToVisionTargetTranslation(target, cameraHeight, cameraPitch, Constants.kBottomVisionTargetHeight);
    }

    private static Translation2d getCameraToVisionTargetTranslation(TargetInfo target, double cameraHeight, Rotation2d cameraPitch, double targetCornerHeight) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(cameraPitch);
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = targetCornerHeight - cameraHeight;
        if ((z > 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }
        return null;
    }

    private void updateGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetTranslations, GoalTracker tracker, Limelight source) {
        if (cameraToVisionTargetTranslations.size() != 2 ||
                cameraToVisionTargetTranslations.get(0) == null ||
                cameraToVisionTargetTranslations.get(1) == null) {
            return;
        }

        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetTranslations.get(0).interpolate(
                cameraToVisionTargetTranslations.get(1), 0.5));

        Pose2d fieldToVisionTarget = getFieldToTurret(timestamp).transformBy(source.getTurretToLens()).transformBy(cameraToVisionTarget);

        if (fieldToVisionTarget.getTranslation().direction().cos() < 0.0) {
            return;
        }

        // Goal normal is always oriented at 180 deg.
        tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), Rotation2d.fromDegrees(180.0))));
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Limelight source) {
        List<Translation2d> cameraToVisionTargetTranslations = new ArrayList<>();

        if (observations == null || observations.isEmpty()) {
            goal_tracker_.maybePruneTracks();
            return;
        }

        for (TargetInfo target : observations) {
            cameraToVisionTargetTranslations.add(getCameraToVisionTargetTranslation(target, source));
        }

        updateGoalTracker(timestamp, cameraToVisionTargetTranslations, goal_tracker_, source);
    }

    public synchronized Pose2d getFieldToVisionTarget() {
        GoalTracker tracker = goal_tracker_;

        if (!tracker.hasTracks()) {
            return null;
        }

        return tracker.getTracks().get(0).field_to_target;
    }

    public synchronized Pose2d getVehicleToVisionTarget(double timestamp) {
        Pose2d fieldToVisionTarget = getFieldToVisionTarget();

        if (fieldToVisionTarget == null) {
            return null;
        }

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget);
    }

    public synchronized Optional<AimingParameters> getAimingParameters(int prev_track_id, double max_track_age, Pose2d target_to_goal_offset) {
        GoalTracker tracker = goal_tracker_;
        List<GoalTracker.TrackReport> reports = tracker.getTracks();

        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double timestamp = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = new TrackReportComparator(
                Constants.kTrackStabilityWeight,
                Constants.kTrackAgeWeight,
                Constants.kTrackSwitchingWeight,
                prev_track_id, timestamp);
        reports.sort(comparator);

        GoalTracker.TrackReport report = null;
        for (GoalTracker.TrackReport track : reports) {
            if (track.latest_timestamp > timestamp - max_track_age) {
                report = track;
                break;
            }
        }
        if (report == null) {
            return Optional.empty();
        }

        AimingParameters params = new AimingParameters(getFieldToVehicle(timestamp),
                report.field_to_target.transformBy(target_to_goal_offset),
                report.latest_timestamp, report.stability, report.id);
        return Optional.of(params);
    }

    public Pose2d getRobot() {
        return new Pose2d();
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());

        SmartDashboard.putString("Field To Robot", getLatestFieldToVehicle().getValue().toString());

        if (getVehicleToVisionTarget(Timer.getFPGATimestamp()) != null) {
            SmartDashboard.putString("Robot to Vision Target", getVehicleToVisionTarget(Timer.getFPGATimestamp()).toString());
        }
    }
}