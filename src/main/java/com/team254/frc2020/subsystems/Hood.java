package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.team254.frc2020.Constants;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class Hood extends ServoMotorSubsystem {
    private static Hood mInstance;
    private LED mLED = LED.getInstance();
    private LatchedBoolean mWasHoming = new LatchedBoolean();
    private double mHomingStartTime = Double.NaN;
    private double mHomingLimitStartTime = Double.NaN;
    private boolean mHoming = false;

    private static final double kMaxHomingTime = 5.0;
    private static final double kHomingHardstopTime = 1.0;

    public synchronized static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood(Constants.kHoodConstants);
        }

        return mInstance;
    }

    private Hood(final ServoMotorSubsystemConstants constants) {
        super(constants);

        TalonUtil.checkError(
                mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen),
                mConstants.kName + ": Could not set reverse limit switch: ");
        mMaster.overrideLimitSwitchesEnable(true);
    }

    public synchronized void setHoming() {
        mHoming = true;
        mHomingStartTime = Timer.getFPGATimestamp();
        mHomingLimitStartTime = Double.NaN;
        mHasBeenZeroed = false;

        mMaster.overrideLimitSwitchesEnable(false);
        TalonUtil.checkError(mMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                        true,
                        5,
                        10,
                        mConstants.kStatorPeakCurrentDuration)),
                mConstants.kName + ": Could not set stator current limit.");
        mLED.setHoodFault();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mWasHoming.update(!hasBeenZeroed())) {
            setHoming();
            System.out.println("Hood needs homing! Entering homing mode.");
        }

        if (mHoming) {
            if ((Timer.getFPGATimestamp() - mHomingStartTime) > kMaxHomingTime) {
                System.out.println("Timeout on hood homing! Zeroing!");
                zeroSensors();
            }

            mMaster.set(ControlMode.PercentOutput, -0.05,
                    DemandType.ArbitraryFeedForward, 0.0);
            if (atHomingLocation() && Double.isNaN(mHomingLimitStartTime)) {
                mHomingLimitStartTime = Timer.getFPGATimestamp();
            } else if (!atHomingLocation()) {
                mHomingLimitStartTime = Double.NaN;
            }
            if (!Double.isNaN(mHomingLimitStartTime) &&
                    (Timer.getFPGATimestamp() - mHomingLimitStartTime) > kHomingHardstopTime) {
                resetIfAtHome();
            }

            if (hasBeenZeroed()) {
                System.out.println("Hood homed successfully!");
                mLED.clearHoodFault();
                mMaster.overrideLimitSwitchesEnable(true);
                TalonUtil.checkError(mMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                                mConstants.kEnableStatorCurrentLimit,
                                mConstants.kStatorContinuousCurrentLimit,
                                mConstants.kStatorPeakCurrentLimit,
                                mConstants.kStatorPeakCurrentDuration)),
                        mConstants.kName + ": Could not set stator current limit.");
                mMaster.set(ControlMode.PercentOutput, 0.0,
                        DemandType.ArbitraryFeedForward, 0.0);

                mHoming = false;
            }
        } else {
            super.writePeriodicOutputs();
        }
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean isAtSetpoint() {
        return Util.epsilonEquals(mPeriodicIO.position_ticks, mPeriodicIO.demand, mConstants.kPositionDeadband);
    }

    @Override
    public boolean atHomingLocation() {
        return mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}