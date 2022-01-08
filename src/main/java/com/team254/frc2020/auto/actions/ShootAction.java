package com.team254.frc2020.auto.actions;

import com.team254.frc2020.subsystems.Superstructure;
import com.team254.lib.util.ShootingParameters;
import edu.wpi.first.wpilibj.Timer;

public class ShootAction implements Action {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final ShootingParameters mShootingParameters;

    private double mShotDuration;
    private double mStartTime = 0.0;
    private boolean mHasStarted = false;

    public ShootAction(ShootingParameters params, double duration) {
        mShootingParameters = params;
        mShotDuration = duration;
    }

    @Override
    public void start() {
        mSuperstructure.setShootingParams(mShootingParameters);
        mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
    }

    @Override
    public void update() {
        if (mSuperstructure.getSystemState() == Superstructure.SystemState.SHOOT && !mHasStarted) {
            mStartTime = Timer.getFPGATimestamp();
            mHasStarted = true;
        }
    }


    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - mStartTime) >= mShotDuration && mHasStarted;
    }

    @Override
    public void done() {
        mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
    }
}