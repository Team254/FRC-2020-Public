package com.team254.lib.util;

/**
 * A drivetrain command consisting of the left, right motor settings and whether
 * the brake mode is enabled.
 */
public class DriveSignal {
    private final double mLeftMotor;
    private final double mRightMotor;
    private final boolean mBrakeMode;

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public static final DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static final DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft() {
        return mLeftMotor;
    }

    public double getRight() {
        return mRightMotor;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    /**
     * @return a new DriveSignal object with the outputs normalized so the max motor
     * output is 1.0
     */
    public DriveSignal normalize() {
        // if either of the left or right signals is greater than 1, creating a scaling
        // factor so that we can proportionally scale down the motor outputs so the max
        // output is 1.0
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(this.getLeft()), Math.abs(this.getRight())));

        // divide by scaling factor so that the max motor output is 1
        return new DriveSignal(this.getLeft() / scaling_factor, this.getRight() / scaling_factor);
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}