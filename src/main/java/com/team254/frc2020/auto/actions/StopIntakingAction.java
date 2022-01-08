package com.team254.frc2020.auto.actions;

import com.team254.frc2020.subsystems.Intake;

public class StopIntakingAction implements Action {
    private final Intake mIntake = Intake.getInstance();

    public StopIntakingAction() {}

    @Override
    public void start() {
        mIntake.setWantedState(Intake.WantedState.IDLE);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}