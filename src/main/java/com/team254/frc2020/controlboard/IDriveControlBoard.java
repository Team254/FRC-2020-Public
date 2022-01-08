package com.team254.frc2020.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getQuickTurn();

    boolean getWantsLowGear();

    boolean getShoot();
}