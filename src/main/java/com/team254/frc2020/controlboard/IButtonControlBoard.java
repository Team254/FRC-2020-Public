package com.team254.frc2020.controlboard;

public interface IButtonControlBoard {
    boolean getAimCoarse(); // 2 pt

    boolean getAimFine(); // 3 pt

    boolean getMoveToZero();

    CardinalDirection getTurretHint();

    void reset();

    boolean getIntake();

    boolean getExhaust();

    boolean getDeployIntake();

    boolean getRetractIntake();

    boolean getHumanPlayerIntake();

    double getTurretJog();

    double getClimbJog();

    double getHoodJog();

    double getStir();

    boolean getCancelAutoSerialize();

    boolean getFnKey();

    boolean getToggleWOFMode();

    boolean getToggleHangMode();

    boolean getZeroGyro();

    boolean getToggleInPitHangMode();
}