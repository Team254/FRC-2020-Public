package com.team254.frc2020;

import com.team254.frc2020.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        LEFT, RIGHT, CENTER
    }

    enum DesiredMode {
        DO_NOTHING, TEST_TRAJECTORY,
        FAR_WOF_8_BALL, FAR_WOF_10_BALL, FAR_WOF_11_BALL,
        NEAR_WOF_10_BALL
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Left", StartingPosition.LEFT);
        mStartPositionChooser.addOption("Right", StartingPosition.RIGHT);
        mStartPositionChooser.addOption("Center", StartingPosition.CENTER);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Trajectory", DesiredMode.TEST_TRAJECTORY);
        mModeChooser.addOption("Far WOF 8 Ball", DesiredMode.FAR_WOF_8_BALL);
        mModeChooser.addOption("Far WOF 10 Ball", DesiredMode.FAR_WOF_10_BALL);
        mModeChooser.addOption("Far WOF 11 Ball", DesiredMode.FAR_WOF_11_BALL);
        mModeChooser.addOption("Near WOF 10 Ball", DesiredMode.NEAR_WOF_10_BALL);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();

        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }

        if (startingPosition == null) {
            startingPosition = StartingPosition.LEFT;
        }

        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private boolean startingLeft(StartingPosition position) {
        return position == StartingPosition.LEFT;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingAutoMode());
            case TEST_TRAJECTORY:
                return Optional.of(new TestTrajectoryFollowingMode());
            case FAR_WOF_8_BALL:
                return Optional.of(new FarWOF8Ball());
            case FAR_WOF_10_BALL:
                return Optional.of(new FarWOF10Ball());
            case NEAR_WOF_10_BALL:
                return Optional.of(new NearWOF10Ball());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}