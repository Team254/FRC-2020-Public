package com.team254.frc2020.states;

public class LEDState {
    public static final LEDState kOff = new LEDState(0.0, 0.0, 0.0);

    public static final LEDState kAiming = new LEDState(0.0, 1.0, 1.0);
    public static final LEDState kShooting = new LEDState(1.0, 0.0, 0.0);

    public static final LEDState kRobotZeroed = new LEDState(0.0, 1.0, 0.0);
    public static final LEDState kFault = new LEDState(0.0, 0.0, 1.0);
    public static final LEDState kFaultHood = new LEDState(1.0, 0.0, 1.0);

    public static final LEDState kClimbing = new LEDState(0.0, 0.3, 1.0);
    public static final LEDState kBrakeEngaged = new LEDState(1.0, 0.0, 0.0);

    public static final LEDState kWOF = new LEDState(1.0, 0.3, 0.0);

    public LEDState() {}

    public LEDState(double b, double g, double r) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LEDState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public double blue;
    public double green;
    public double red;
}
