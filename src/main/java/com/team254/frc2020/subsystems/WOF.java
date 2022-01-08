package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

public class WOF extends Subsystem {
    private static WOF mInstance;

    private final TalonFX mSpinnerMaster;
    private final Solenoid mDeploySolenoid;

    private boolean mGameDataObtained = false;

    private final boolean kDeployValue = true;

    private SimpleWidget colorWidget;

    private boolean mIsDeployed;

    public static class PeriodicIO {
        double demand = 0.0;
    }

    PeriodicIO mPeriodicIO = new PeriodicIO();

    private WOF() {
        mSpinnerMaster = TalonFXFactory.createDefaultTalon(Constants.kWOFSpinnerId);
        mDeploySolenoid = new Solenoid(Constants.kPCMId, Constants.kWOFSolenoidId);

        // force a CAN message
        mSpinnerMaster.setNeutralMode(NeutralMode.Brake);
        mIsDeployed = true;
        setDeploy(false);

        colorWidget = Shuffleboard.getTab("Color").add("Position Control Color", false).withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when false", "black"));
    }

    public static WOF getInstance() {
        if (mInstance == null) {
            mInstance = new WOF();
        }
        return mInstance;
    }

    @Override
    public void readPeriodicInputs() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0 && !mGameDataObtained) {
            mGameDataObtained = true;
            String color;
            switch (gameData) {
                case "B":
                    color = "blue";
                    break;
                case "G":
                    color = "green";
                    break;
                case "R":
                    color = "red";
                    break;
                case "Y":
                    color = "yellow";
                    break;
                default:
                    color = "black";
                    break;
            }

            colorWidget.withProperties(Map.of("color when true", color));
            colorWidget.getEntry().setBoolean(true);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void setDeploy(boolean should_deploy) {
        if (mIsDeployed != should_deploy) {
            mDeploySolenoid.set(should_deploy);
            mIsDeployed = should_deploy;
        }
    }

    public synchronized void setOpenLoop(double demand) {
        mPeriodicIO.demand = demand;
        if (!mIsDeployed) {
            mPeriodicIO.demand = 0.0;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mSpinnerMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }

    @Override
    public void stop() {
        mPeriodicIO.demand = 0.0;
        mSpinnerMaster.set(ControlMode.PercentOutput, 0.0);
        setDeploy(false);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("WOF Solenoid Deployed", mIsDeployed);
        SmartDashboard.putNumber("WOF Demand", mPeriodicIO.demand);

    }
}

