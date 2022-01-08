package com.team254.lib.util;

import com.team254.frc2020.Constants;
import com.team254.frc2020.Kinematics;
import com.team254.lib.geometry.Twist2d;

public class OpenLoopCheesyDriveHelper {
    private static OpenLoopCheesyDriveHelper mInstance;

    public static OpenLoopCheesyDriveHelper getInstance() {
        if (mInstance == null) {
            mInstance = new OpenLoopCheesyDriveHelper();
        }

        return mInstance;
    }

    private OpenLoopCheesyDriveHelper() {}

    private final double kWheelGain = 0.05;
    private final double kWheelNonlinearity = 0.05;
    private final double kDenominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);

    public synchronized DriveSignal cheesyDrive(double throttle, double wheel, boolean quickTurn) {

        throttle = Util.handleDeadband(throttle, Constants.kDriveThrottleDeadband);
        wheel = Util.handleDeadband(wheel, Constants.kDriveWheelDeadband);

        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (kDenominator * kDenominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        return new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor);
    }
}