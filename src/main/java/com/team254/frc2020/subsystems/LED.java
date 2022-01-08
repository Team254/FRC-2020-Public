package com.team254.frc2020.subsystems;

import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.frc2020.states.LEDState;
import com.team254.frc2020.states.TimedLEDState;

public class LED extends Subsystem {
    private boolean mTurretFault = false;
    private boolean mHoodFault = false;

    public enum WantedAction {
        DISPLAY_FAULT,
        DISPLAY_CLIMB,
        DISPLAY_SUPERSTRUCTURE,
        DISPLAY_WOF,
        DISPLAY_ZEROED,
    }

    private enum SystemState {
        DISPLAYING_FAULT,
        DISPLAYING_CLIMBING,
        DISPLAYING_SUPERSTRUCTURE,
        DISPLAYING_WOF,
        DISPLAYING_ZEROED,
    }

    private static LED mInstance;

    private Canifier mCanifier;
    private SystemState mSystemState = SystemState.DISPLAYING_SUPERSTRUCTURE;
    private WantedAction mWantedAction = WantedAction.DISPLAY_SUPERSTRUCTURE;

    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);
    private TimedLEDState mSuperstructureLEDState = TimedLEDState.StaticLEDState.kStaticOff;
    private TimedLEDState mClimbLEDState = TimedLEDState.StaticLEDState.kStaticOff;
    private TimedLEDState mWOFLEDState = TimedLEDState.StaticLEDState.kStaticOff;

    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private LED() {
        mCanifier = Canifier.getInstance();
    }

    public synchronized void setTurretFault() {
        mTurretFault = true;
    }

    public synchronized void clearTurretFault() {
        mTurretFault = false;
    }

    public synchronized void setHoodFault() {
        mHoodFault = true;
    }

    public synchronized void clearHoodFault() {
        mHoodFault = false;
    }

    public synchronized void setSuperstructureLEDState(TimedLEDState intakeLEDState) {
        mSuperstructureLEDState = intakeLEDState;
    }

    public synchronized void setClimbLEDState(TimedLEDState climbLEDState) {
        mClimbLEDState = climbLEDState;
    }

    public synchronized void setWOFLEDState(TimedLEDState climbLEDState) {
        mWOFLEDState = climbLEDState;
    }

    public synchronized void setWantedAction(WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            double stateStartTime;

            @Override
            public void onStart(double timestamp) {
                stateStartTime = timestamp;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {
                    SystemState newState = getStateTransition();

                    if (mSystemState != newState) {
                        System.out.println(timestamp + ": LED changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        stateStartTime = timestamp;
                    }

                    double timeInState = timestamp - stateStartTime;

                    switch (mSystemState) {
                        case DISPLAYING_SUPERSTRUCTURE:
                            setSuperstructureLEDCommand(timeInState);
                            break;
                        case DISPLAYING_FAULT:
                            setFaultLEDCommand(timeInState);
                            break;
                        case DISPLAYING_CLIMBING:
                            setHangLEDCommand(timeInState);
                            break;
                        case DISPLAYING_WOF:
                            setWOFLEDCommand(timeInState);
                            break;
                        case DISPLAYING_ZEROED:
                            setZeroedCommand(timeInState);
                            break;
                        default:
                            System.out.println("Fell through on LED commands: " + mSystemState);
                            break;
                    }
                    mCanifier.setLEDColor(mDesiredLEDState.red, mDesiredLEDState.green,
                            mDesiredLEDState.blue);
                }
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    private void setZeroedCommand(double timeInState) {
        TimedLEDState.StaticLEDState.kRobotZeroed.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setWOFLEDCommand(double timeInState) {
        mWOFLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setSuperstructureLEDCommand(double timeInState) {
        mSuperstructureLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setFaultLEDCommand(double timeInState) {
        if (mTurretFault) {
            TimedLEDState.BlinkingLEDState.kZeroingFault.getCurrentLEDState(mDesiredLEDState, timeInState);
        } else {
            TimedLEDState.BlinkingLEDState.kZeroingHoodFault.getCurrentLEDState(mDesiredLEDState,
                    timeInState);
        }
    }

    private void setHangLEDCommand(double timeInState) {
        mClimbLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private SystemState getStateTransition() {
        if (mHoodFault || mTurretFault) {
            return SystemState.DISPLAYING_FAULT;
        }
        switch (mWantedAction) {
            case DISPLAY_SUPERSTRUCTURE:
                return SystemState.DISPLAYING_SUPERSTRUCTURE;
            case DISPLAY_CLIMB:
                return SystemState.DISPLAYING_CLIMBING;
            case DISPLAY_WOF:
                return SystemState.DISPLAYING_WOF;
            case DISPLAY_FAULT:
                return SystemState.DISPLAYING_FAULT;
            case DISPLAY_ZEROED:
                return SystemState.DISPLAYING_ZEROED;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.DISPLAYING_SUPERSTRUCTURE;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
