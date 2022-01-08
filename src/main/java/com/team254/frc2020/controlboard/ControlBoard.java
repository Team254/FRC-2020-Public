package com.team254.frc2020.controlboard;

import com.team254.frc2020.Constants;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mDriveControlBoard = Constants.kUseDriveGamepad ? GamepadDriveControlBoard.getInstance()
                : MainDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getWantsLowGear() {
        return mDriveControlBoard.getWantsLowGear();
    }

    @Override
    public boolean getShoot() {
        return mDriveControlBoard.getShoot();
    }

    @Override
    public boolean getAimCoarse() {
        return mButtonControlBoard.getAimCoarse();
    }

    @Override
    public boolean getAimFine() {
        return mButtonControlBoard.getAimFine();
    }

    @Override
    public boolean getMoveToZero() {
        return mButtonControlBoard.getMoveToZero();
    }

    @Override
    public CardinalDirection getTurretHint() {
        return mButtonControlBoard.getTurretHint();
    }

    @Override
    public void reset() {
        mButtonControlBoard.reset();
    }

    @Override
    public boolean getIntake() {
        return mButtonControlBoard.getIntake();
    }

    @Override
    public boolean getExhaust() {
        return mButtonControlBoard.getExhaust();
    }

    @Override
    public boolean getDeployIntake() {
        return mButtonControlBoard.getDeployIntake();
    }

    @Override
    public boolean getRetractIntake() {
        return mButtonControlBoard.getRetractIntake();
    }

    @Override
    public boolean getHumanPlayerIntake() {
        return mButtonControlBoard.getHumanPlayerIntake();
    }

    @Override
    public double getTurretJog() {
        return mButtonControlBoard.getTurretJog();
    }

    @Override
    public double getClimbJog() {
        return mButtonControlBoard.getClimbJog();
    }

    @Override
    public double getHoodJog() {
        return mButtonControlBoard.getHoodJog();
    }

    @Override
    public double getStir() {
        return mButtonControlBoard.getStir();
    }

    @Override
    public boolean getCancelAutoSerialize() {
        return mButtonControlBoard.getCancelAutoSerialize();
    }

    @Override
    public boolean getFnKey() {
        return mButtonControlBoard.getFnKey();
    }

    @Override
    public boolean getToggleWOFMode() {
        return mButtonControlBoard.getToggleWOFMode();
    }

    @Override
    public boolean getToggleHangMode() {
        return mButtonControlBoard.getToggleHangMode();
    }

    @Override
    public boolean getZeroGyro() {
        return mButtonControlBoard.getZeroGyro();
    }

    @Override
    public boolean getToggleInPitHangMode() {
        return mButtonControlBoard.getToggleInPitHangMode();
    }

}