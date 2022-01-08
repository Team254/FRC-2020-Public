package com.team254.frc2020.controlboard;

import com.team254.frc2020.Constants;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard mInstance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadDriveControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadDriveControlBoard() {
        mController = new XboxController(Constants.kDriveGamepadPort);
    }

    @Override
    public double getThrottle() {
        return mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    @Override
    public double getTurn() {
        return -mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    @Override
    public boolean getWantsLowGear() {
        return mController.getButton(XboxController.Button.LB);
    }

    @Override
    public boolean getQuickTurn() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getShoot() {
        return mController.getTrigger(XboxController.Side.LEFT);
    }
}