package com.team254.frc2020.controlboard;

import com.team254.frc2020.Constants;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.Timer;

public class GamepadButtonControlBoard implements IButtonControlBoard {

    private static GamepadButtonControlBoard mInstance = null;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;
    private CardinalDirection mLastCardinal;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kButtonGamepadPort);
    }

    @Override
    public boolean getAimCoarse() {
        return mController.getTrigger(XboxController.Side.LEFT) && !getFnKey();
    }

    @Override
    public boolean getAimFine() {
        return mController.getTrigger(XboxController.Side.RIGHT) && !getFnKey();
    }

    @Override
    public boolean getMoveToZero() {
        return mController.getButton(XboxController.Button.A) && getFnKey();
    }

    @Override
    public CardinalDirection getTurretHint() {
        int dPad = mController.getDPad();
        CardinalDirection newCardinal = dPad == -1 ? CardinalDirection.NONE : CardinalDirection.findClosest(Rotation2d.fromDegrees(-dPad));
        if (newCardinal != CardinalDirection.NONE && CardinalDirection.isDiagonal(newCardinal)) {
            // Latch previous direction on diagonal presses, because the D-pad sucks at diagonals.
            newCardinal = mLastCardinal;
        }
        boolean valid = mDPadValid.update(Timer.getFPGATimestamp(), newCardinal != CardinalDirection.NONE && (mLastCardinal == CardinalDirection.NONE || newCardinal == mLastCardinal));
        if (valid) {
            if (mLastCardinal == CardinalDirection.NONE) {
                mLastCardinal = newCardinal;
            }
            return mLastCardinal;
        } else {
            mLastCardinal = newCardinal;
        }
        return CardinalDirection.NONE;
    }

    @Override
    public boolean getIntake() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getExhaust() {
        return mController.getButton(XboxController.Button.LB);
    }

    @Override
    public boolean getDeployIntake() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public boolean getRetractIntake() {
        return mController.getButton(XboxController.Button.Y) && !getFnKey();
    }

    @Override
    public boolean getHumanPlayerIntake() {
        return mController.getButton(XboxController.Button.A) && !getFnKey();
    }

    @Override
    public double getTurretJog() {
        return -mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
    }

    @Override
    public double getClimbJog() {
        return mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.Y);
    }

    @Override
    public double getHoodJog() {
        return mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y);
    }

    @Override
    public double getStir() {
        return mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
    }

    @Override
    public boolean getCancelAutoSerialize() {
        return mController.getButton(XboxController.Button.R_JOYSTICK);
    }

    @Override
    public void reset() {
        mLastCardinal = CardinalDirection.NONE;
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    @Override
    public boolean getFnKey() {
        return mController.getButton(XboxController.Button.BACK);
    }

    @Override
    public boolean getToggleWOFMode() {
        return mController.getButton(XboxController.Button.X) && getFnKey();
    }

    @Override
    public boolean getToggleHangMode() {
        return mController.getButton(XboxController.Button.START) && getFnKey();
    }

    @Override
    public boolean getZeroGyro() {
        return mController.getButton(XboxController.Button.Y) && getFnKey();
    }

    @Override
    public boolean getToggleInPitHangMode() {
        return getFnKey() && mController.getTrigger(XboxController.Side.LEFT) && mController.getTrigger(XboxController.Side.RIGHT);
    }
}