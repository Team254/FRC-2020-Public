package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.team254.frc2020.Constants;
import com.team254.lib.drivers.TalonUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends ServoMotorSubsystem {
    private static Turret mInstance;
    private Canifier mCanifier;

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret(Constants.kTurretConstants);
        }

        return mInstance;
    }

    private Turret(final ServoMotorSubsystemConstants constants) {
        super(constants);
        mCanifier = Canifier.getInstance();
        TalonUtil.checkError(
                mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen),
                mConstants.kName + ": Could not set forward limit switch: ");

        TalonUtil.checkError(
                mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen),
                mConstants.kName + ": Could not set reverse limit switch: ");

        mMaster.overrideLimitSwitchesEnable(true);

    }

    @Override
    public boolean atHomingLocation() {
        return mCanifier.isTurretHomed();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Turret Reverse Limit Switch", mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1);
        SmartDashboard.putBoolean("Turret Forward Limit Switch", mMaster.getSensorCollection().isFwdLimitSwitchClosed() == 1);
        super.outputTelemetry();
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}