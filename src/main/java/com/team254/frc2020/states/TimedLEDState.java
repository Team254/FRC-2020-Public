package com.team254.frc2020.states;

public interface TimedLEDState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        public static BlinkingLEDState kZeroingFault = new BlinkingLEDState(
                LEDState.kOff, LEDState.kFault, 1.0);
        public static BlinkingLEDState kZeroingHoodFault = new BlinkingLEDState(
                LEDState.kOff, LEDState.kFaultHood, 1.0);

        public static BlinkingLEDState kClimbing = new BlinkingLEDState(
                LEDState.kOff, LEDState.kClimbing, 0.5);
        public static BlinkingLEDState kBrakeEngaged = new BlinkingLEDState(
                LEDState.kOff, LEDState.kBrakeEngaged, 0.5);

        public static BlinkingLEDState kBlinkingAiming = new BlinkingLEDState(
                LEDState.kOff, LEDState.kAiming, 0.1);
        public static BlinkingLEDState kBlinkingShooting = new BlinkingLEDState(
                LEDState.kOff, LEDState.kShooting, 0.1);

        public static BlinkingLEDState kBlinkingWOF = new BlinkingLEDState(
                LEDState.kOff, LEDState.kWOF, 0.5);


        LEDState mStateOne = new LEDState(0.0, 0.0, 0.0);
        LEDState mStateTwo = new LEDState(0.0, 0.0, 0.0);
        double mDuration;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if ((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }
        }
    }

    class StaticLEDState implements TimedLEDState {
        public static StaticLEDState kStaticOff = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kRobotZeroed = new StaticLEDState(LEDState.kRobotZeroed);


        LEDState mStaticState = new LEDState(0.0, 0.0, 0.0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }
}
