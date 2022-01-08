package com.team254.frc2020.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.team254.frc2020.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Canifier extends Subsystem {

    private static Canifier mInstance;

    public synchronized static Canifier getInstance() {
        if (mInstance == null) {
            mInstance = new Canifier();
        }
        return mInstance;
    }

    private PeriodicIO mPeriodicIO;
    private boolean mOutputsChanged;

    public static class PeriodicIO {
        boolean break_beam_triggered = false;
        boolean turret_homing_limit_switch = false;

        double g = 0.0;
        double r = 0.0;
        double b = 0.0;
    }

    private CANifier mCanifier;

    private Canifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 10, Constants.kLongCANTimeoutMs);

        mPeriodicIO = new PeriodicIO();

        // Force a first update.
        mOutputsChanged = true;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.break_beam_triggered = mCanifier.getGeneralInput(CANifier.GeneralPin.LIMR);
        mPeriodicIO.turret_homing_limit_switch = !mCanifier.getGeneralInput(CANifier.GeneralPin.LIMF);
    }

    public synchronized void setLEDColor(double red, double green, double blue) {
        if (red != mPeriodicIO.r ||
                green != mPeriodicIO.g ||
                blue != mPeriodicIO.b) {
            mPeriodicIO.r = red;
            mPeriodicIO.g = green;
            mPeriodicIO.b = blue;
            mOutputsChanged = true;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // A: Green
        // B: Red
        // C: Blue
        if (mOutputsChanged) {
            mCanifier.setLEDOutput(mPeriodicIO.g, CANifier.LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(mPeriodicIO.r, CANifier.LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(mPeriodicIO.b, CANifier.LEDChannel.LEDChannelC);
            mOutputsChanged = false;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Turret Homing Limit Switch", mPeriodicIO.turret_homing_limit_switch);
    }

    public synchronized boolean isBreamBeamSensorTriggered() {
        return mPeriodicIO.break_beam_triggered;
    }

    public synchronized boolean isTurretHomed() {
        return mPeriodicIO.turret_homing_limit_switch;
    }
}
