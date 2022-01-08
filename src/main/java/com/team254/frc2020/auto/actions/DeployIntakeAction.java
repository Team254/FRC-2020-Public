package com.team254.frc2020.auto.actions;

import com.team254.frc2020.subsystems.Intake;

public class DeployIntakeAction implements Action {
    private final Intake mIntake = Intake.getInstance();

    private final boolean mShouldDeployIntake;

    public DeployIntakeAction(boolean should_deploy) {
        mShouldDeployIntake = should_deploy;
    }

    @Override
    public void start() {
        if (mShouldDeployIntake) {
            mIntake.deploy();
        } else {
            mIntake.stow();
        }
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return mIntake.isStowed() == !mShouldDeployIntake;
    }

    @Override
    public void done() {}
}