package com.team254.frc2020.subsystems;

import com.team254.frc2020.Constants;
import com.team254.frc2020.RobotState;
import com.team254.frc2020.limelight.PipelineConfiguration;
import com.team254.frc2020.limelight.constants.LimelightConstants;
import com.team254.frc2020.limelight.undistort.UndistortMap;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight extends Subsystem {

    private final NetworkTable mNetworkTable;

    public Limelight(LimelightConstants constants, PipelineConfiguration pipelineConfig) {
        mConstants = constants;
        setPipelineConfig(pipelineConfig);
        mNetworkTable = NetworkTableInstance.getDefault().getTable(constants.getTableName());
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    private LimelightConstants mConstants;
    private PipelineConfiguration mPipelineConfig;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputsHaveChanged = true;
    private final double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
    private final List<TargetInfo> mTargets = new ArrayList<>();
    private boolean mSeesTarget = false;

    public Pose2d getTurretToLens() {
        return mConstants.getTurretToLens();
    }

    public double getLensHeight() {
        return mConstants.getHeight();
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return mConstants.getHorizontalPlaneToLens();
    }

    public void setPipelineConfig(PipelineConfiguration pipelineConfig) {
        mPipelineConfig = pipelineConfig;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.kImageCaptureLatency;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode ||
                mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                setLed(Limelight.LedMode.OFF);
                RobotState.getInstance().resetVision();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Limelight.this) {
                    RobotState.getInstance().addVisionUpdate(
                            timestamp - getLatency(),
                            getTarget(), Limelight.this);
                }

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };
        mEnabledLooper.register(mLoop);
    }

    @Override
    public void stop() {
        setLed(Limelight.LedMode.OFF);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean(mConstants.getName() + ": Has Target", mSeesTarget);
        SmartDashboard.putNumber(mConstants.getName() + ": Pipeline Latency (ms)", mPeriodicIO.latency);
        SmartDashboard.putNumber(mConstants.getName() + ": LED Mode", mPeriodicIO.ledMode);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipelineNumber(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            RobotState.getInstance().resetVision();
            mPeriodicIO.pipeline = mode;

            System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = getRawTargetInfos();

        if (mSeesTarget && targets != null) {
            return targets;
        }

        return null;
    }

    public synchronized List<TargetInfo> getRawTargetInfos() {
        return getRawTargetInfos(
                Constants.kUseTopCorners ? getTopCorners() : getBottomCorners(),
                mPipelineConfig, mTargets, mConstants.getUndistortMap(), mConstants.getHorizontalFOV(), mConstants.getVerticalFOV());
    }

    public static List<TargetInfo> getRawTargetInfos(List<double[]> corners, PipelineConfiguration pipeline, List<TargetInfo> targets,
                                                     UndistortMap undistortMap, double kHorizontalFOV, double kVerticalFOV) {
        if (corners == null) {
            return null;
        }

        List<double[]> transformedCorners = new ArrayList<>(2);
        for (int i = 0; i < 2; i++) {
            double[] corner = corners.get(i);
            corner = pipeline.normalize(corner);
            if (Constants.kShouldUndistort) {
                double[] undistorted = undistortMap.getUndistortedPoint(corner[0], corner[1]);
                corner[0] = undistorted[0];
                corner[1] = undistorted[1];
            }
            transformedCorners.add(corner);
        }

        double slope = 0.0;
        if (Math.abs(transformedCorners.get(1)[0] - transformedCorners.get(0)[0]) > Util.kEpsilon) {
            slope = (transformedCorners.get(1)[1] - transformedCorners.get(0)[1]) /
                    (transformedCorners.get(1)[0] - transformedCorners.get(0)[0]);
        }

        targets.clear();

        double VPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
        double VPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
        for (int i = 0; i < 2; ++i) {
            // Average of y and z;
            double y_pixels = transformedCorners.get(i)[0];
            double z_pixels = transformedCorners.get(i)[1];

            // Redefine to robot frame of reference.
            double nY = -(y_pixels * 2 - 1);
            double nZ = -(z_pixels * 2 - 1);

            double y = VPW / 2 * nY;
            double z = VPH / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            targets.add(target);
        }

        return targets;
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private List<double[]> getTopCorners() {
        double[] corners = mNetworkTable.getEntry("tcornxy").getDoubleArray(mZeroArray);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!mSeesTarget || corners.length < 8 || corners == mZeroArray || corners.length % 2 != 0) {
            return null;
        }

        double[] xCorners = new double[corners.length / 2];
        double[] yCorners = new double[corners.length / 2];

        for (int i = 0; i < corners.length; i++) {
            if (i % 2 == 0) {
                xCorners[i / 2] = corners[i];
            } else {
                yCorners[i / 2] = corners[i];
            }
        }

        return extractTopCornersFromBoundingBoxes(xCorners, yCorners);
    }

    private List<double[]> getBottomCorners() {
        double[] corners = mNetworkTable.getEntry("tcornxy").getDoubleArray(mZeroArray);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!mSeesTarget || corners.length != 8 || corners == mZeroArray) {
            return null;
        }

        double[] xCorners = new double[corners.length / 2];
        double[] yCorners = new double[corners.length / 2];

        for (int i = 0; i < corners.length; i++) {
            if (i % 2 == 0) {
                xCorners[i / 2] = corners[i];
            } else {
                yCorners[i / 2] = corners[i];
            }
        }

        return extractBottomCornersFromBoundingBoxes(xCorners, yCorners);
    }

    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::x);
    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        List<Translation2d> left = corners.subList(0, corners.size() / 2);
        List<Translation2d> right = corners.subList(corners.size() / 2, corners.size());

        left.sort(ySort);
        right.sort(ySort);

        List<Translation2d> leftTop = left.subList(0, (corners.size() / 2) / 2);
        List<Translation2d> rightTop = right.subList(0, (corners.size() / 2) / 2);

        leftTop.sort(xSort);
        rightTop.sort(xSort);

        Translation2d leftCorner = leftTop.get(0);
        Translation2d rightCorner = rightTop.get(rightTop.size() - 1);
        return Arrays.asList(new double[]{leftCorner.x(), leftCorner.y()}, new double[]{rightCorner.x(), rightCorner.y()});
    }

    public static List<double[]> extractBottomCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        List<Translation2d> left = corners.subList(0, corners.size() / 2);
        List<Translation2d> right = corners.subList(corners.size() / 2, corners.size());

        left.sort(ySort);
        right.sort(ySort);

        Translation2d leftCorner = left.get(left.size() - 1);
        Translation2d rightCorner = right.get(right.size() - 1);

        return Arrays.asList(new double[]{leftCorner.x(), leftCorner.y()}, new double[]{rightCorner.x(), rightCorner.y()});
    }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    public LimelightConstants getConstants() {
        return mConstants;
    }

    public void setConstants(LimelightConstants mConstants) {
        this.mConstants = mConstants;
    }

}