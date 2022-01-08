package com.team254.frc2020.subsystems;

import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import edu.wpi.first.wpilibj.Compressor;

/**
 * Subsystem to ensure the compressor never runs while the superstructure moves
 */
public class Infrastructure extends Subsystem {
    private static Infrastructure mInstance;

    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Compressor mCompressor = new Compressor(Constants.kPCMId);

    private boolean mIsTeleop = false;

    private Infrastructure() {}

    public static Infrastructure getInstance() {
        if (mInstance == null) {
            mInstance = new Infrastructure();
        }

        return mInstance;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                synchronized (Infrastructure.this) {
                    boolean superstructureMoving = mSuperstructure.getSystemState() == Superstructure.SystemState.SHOOT ||
                            mSuperstructure.getSystemState() == Superstructure.SystemState.AIMING;

                    if (superstructureMoving || !mIsTeleop) {
                        stopCompressor();
                    } else {
                        startCompressor();
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    public synchronized void setIsTeleop(boolean isTeleop) {
        mIsTeleop = isTeleop;

        if (mIsTeleop) {
            startCompressor();
        }
    }

    public synchronized boolean isTeleop() {
        return mIsTeleop;
    }

    private void startCompressor() {
        mCompressor.start();
    }

    private void stopCompressor() {
        mCompressor.stop();
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}
}
