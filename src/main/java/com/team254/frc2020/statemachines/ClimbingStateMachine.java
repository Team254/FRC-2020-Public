package com.team254.frc2020.statemachines;

import com.team254.frc2020.subsystems.Drive;
import com.team254.frc2020.subsystems.LED;
import com.team254.frc2020.states.TimedLEDState;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.atomic.AtomicBoolean;

public class ClimbingStateMachine {
    private static final double kFeedforwardDown = 1.0 / 12.0;
    private static final int kMaxExtension = -294000;
    private static final int kClimbPosition = -100000;//-280000;
    private static final int kDeployPistonPosition = -155196;
    private static final int kDeployPistonHysteresis = 5000; // dont undeploy until falls 5000 below deploy position
    private static final double kThrottleDeadband = 0.4;

    enum SystemState {
        PRECLIMB,
        DISENGAGING_BRAKE,
        EXTENDING,
        MANUAL,
        CLIMBING
    }

    private Drive mDrive;
    private LED mLED;

    private SystemState mSystemState = SystemState.PRECLIMB;
    private double mStateStartTime = Timer.getFPGATimestamp();
    private LatchedBoolean mDeployToggle = new LatchedBoolean();
    private LatchedBoolean mBrakeToggle = new LatchedBoolean();
    private double mBrakeTime = Double.NaN;
    private AtomicBoolean mIsInPitMode = new AtomicBoolean(false);
    private double mCurrentLimit = 60;

    public ClimbingStateMachine() {
        mDrive = Drive.getInstance();
        mLED = LED.getInstance();
    }

    public synchronized void reset() {
        mSystemState = SystemState.PRECLIMB;
        mDrive.setPTOEngaged(false);
        mDrive.setBrakeEngaged(true);
        mDrive.configPTOPID(false);
        mDrive.setDeploy(false);
        mDrive.stop();
        mDeployToggle.update(true);
        mBrakeToggle.update(true);
        mDrive.configPTOCurrentLimits(60);
        mBrakeTime = Double.NaN;
    }

    public synchronized void setInPitMode(boolean enable) {
        mIsInPitMode.set(enable);
    }

    public synchronized void handle(double timestamp, double climbThrottle, boolean climb,
                                    boolean deploy, boolean brakeOn, boolean brakeOff) {
        double timeInState = timestamp - mStateStartTime;
        double wantedCurrentLimit = mCurrentLimit;

        if (mDeployToggle.update(deploy)) {
            mDrive.setDeploy(!mDrive.getDeploy());
        }

        if (brakeOn) {
            mDrive.setBrakeEngaged(true);
            if (Double.isNaN(mBrakeTime)) {
                mBrakeTime = timestamp;
            }
        } else if (brakeOff) {
            mDrive.setBrakeEngaged(false);
            mBrakeTime = Double.NaN;
        }

        if (mDrive.getBrake()) {
            mLED.setClimbLEDState(TimedLEDState.BlinkingLEDState.BlinkingLEDState.kClimbing);
        } else {
            mLED.setClimbLEDState(TimedLEDState.BlinkingLEDState.BlinkingLEDState.kBrakeEngaged);
        }

        boolean stopMotors = false;
        if (mDrive.getBrake() && !Double.isNaN(mBrakeTime) && (timestamp - mBrakeTime > 0.5)) {
            climbThrottle = 0.0;
            stopMotors = true;
        }

        climbThrottle = Util.limit(climbThrottle, 1.0);

        if (mSystemState == SystemState.EXTENDING || mSystemState == SystemState.MANUAL || mSystemState == SystemState.CLIMBING) {
            if (mDrive.getPTOPosition() < kDeployPistonPosition) {
                mDrive.setDeploy(true);
            } else if (mDrive.getPTOPosition() > (kDeployPistonPosition + kDeployPistonHysteresis)) {
                mDrive.setDeploy(false);
            }
        }

        switch (mSystemState) {
            case PRECLIMB:
                mDrive.setPTOEngaged(true);
                mDrive.setBrakeEngaged(false);
                mDrive.zeroPTOMotors();
                wantedCurrentLimit = 20;
                break;
            case DISENGAGING_BRAKE:
                // Positive throttle is downwards.
                if (timeInState > 0.5) {
                    mDrive.setPTOMotorsOpenLoop(0.1, 0.0);
                } else {
                    mDrive.setPTOMotorsOpenLoop(0.0, 0.0);
                }
                break;
            case EXTENDING:
                mDrive.setPTOMotorsPosition(kMaxExtension);
                break;
            case MANUAL:
                if (stopMotors) {
                    mDrive.setPTOMotorsOpenLoop(0.0, 0.0);
                } else if (Util.inRange(climbThrottle, kThrottleDeadband)) {
                    mDrive.setPTOMotorsPosition(mDrive.getPTOPosition());
                } else {
                    mDrive.setPTOMotorsOpenLoop(Util.handleDeadband(-climbThrottle, kThrottleDeadband), 0.0);
                }
                wantedCurrentLimit = 40;
                break;
            case CLIMBING:
                mDrive.setPTOMotorsPosition(kClimbPosition);
                break;
            default:
                break;
        }

        SystemState nextState = mSystemState;
        switch (mSystemState) {
            case PRECLIMB:
                if (mIsInPitMode.get()) {
                    nextState = SystemState.MANUAL;
                } else {
                    nextState = SystemState.DISENGAGING_BRAKE;
                }
                break;
            case DISENGAGING_BRAKE:
                if (timeInState > 1.0) {
                    nextState = SystemState.EXTENDING;
                    mDrive.setDeploy(true);
                    mDrive.configPTOPID(true);
                }
                break;
            case EXTENDING:
                if (!Util.inRange(climbThrottle, kThrottleDeadband)) {
                    nextState = SystemState.MANUAL;
                }
                if (climb) {
                    nextState = SystemState.CLIMBING;
                }
                if ((timeInState > 0.75) && mDrive.getPTOPosition() > -2000) {
                    System.out.println("Retrying to disengage.");
                    nextState = SystemState.DISENGAGING_BRAKE;
                } else {
                    wantedCurrentLimit = 60;
                }
                break;
            case CLIMBING:
                if (!Util.inRange(climbThrottle, kThrottleDeadband)) {
                    nextState = SystemState.MANUAL;
                }
                break;
            case MANUAL:
                if (climb && !mIsInPitMode.get()) {
                    nextState = SystemState.CLIMBING;
                }
                break;
            default:
                break;
        }

        if (nextState != mSystemState) {
            System.out.println("Transitioned from : " + mSystemState + " to " + nextState);
            mSystemState = nextState;
            mStateStartTime = timestamp;
        }

        if (Util.epsilonEquals(wantedCurrentLimit, mCurrentLimit, .1)) {
            mDrive.configPTOCurrentLimits(wantedCurrentLimit);
            mCurrentLimit = wantedCurrentLimit;
        }
    }
}
