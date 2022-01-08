package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Serializer extends Subsystem {

    public static final double kSpinCycleSerializeDemand = 0.45;
    public static final double kSpinCycleRebalancingDemand = 0.75;
    public static double mSpinCycleFeedDemand = 0.75;
    public static final double kRollerDemandFeed = 14000; // ticks/100ms
    public static final double kSpinCycleOscillationTime = 1.2; // seconds before switching dir
    public static final double kTotalCycleTime = 2.0; // 2 switches in direction per cycle

    public static final double kRebalancingTime = 0.5; // seconds
    public static final double kBeamBreakSerializeTime = 0.25; // seconds, time it serializes after ball left beam break

    private boolean mStirOverride = false;

    private static Serializer mInstance;

    public static Serializer getInstance() {
        if (mInstance == null) {
            mInstance = new Serializer();
        }

        return mInstance;
    }

    public static class PeriodicIO {
        // inputs
        double right_roller_velocity = 0.0;
        double left_roller_velocity = 0.0;
        boolean break_beam_triggered = false; //is there a ball between the sensor and tape
        // outputs
        double spin_cycle_demand = 0.0;
        double right_roller_demand = 0.0;
        double left_roller_demand = 0.0;
    }

    public enum WantedState {
        IDLE,
        SERIALIZE,
        PREPARE_TO_SHOOT,
        FEED
    }

    public enum SystemState {
        IDLE,
        SERIALIZE,
        FEED,
        REBALANCING,
        REBALANCED,
    }

    private TalonFX mSpinCycleMaster, mRightRollerMaster, mLeftRollerMaster;
    private Solenoid mSkatePark;
    private Solenoid mChock;
    private Canifier mCanifier;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;

    private boolean mSerializerCanceled = false;

    private double mCurrentStateStartTime = 0.0;
    private double mLastBreakBreakTriggerTime = 0.0;

    private boolean mIsSkateParkDeployed = false;
    private boolean mIsChockDeployed = false;

    private Serializer() {
        mSpinCycleMaster = TalonFXFactory.createDefaultTalon(Constants.kSerializerSpinCycleMasterId);
        mSpinCycleMaster.setInverted(true);
        mSpinCycleMaster.setNeutralMode(NeutralMode.Brake);
        mSpinCycleMaster.configOpenloopRamp(0.0);

        mSpinCycleMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mSpinCycleMaster.enableVoltageCompensation(true);

        mSpinCycleMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0.2), Constants.kLongCANTimeoutMs);
        mSpinCycleMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 50, 0.2), Constants.kLongCANTimeoutMs);

        mRightRollerMaster = TalonFXFactory.createDefaultTalon(Constants.kSerializerRightRollerMasterId);
        mRightRollerMaster.setInverted(true);
        mRightRollerMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kLongCANTimeoutMs);
        mRightRollerMaster.configVelocityMeasurementWindow(16, Constants.kLongCANTimeoutMs);

        mRightRollerMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mRightRollerMaster.enableVoltageCompensation(true);


        mLeftRollerMaster = TalonFXFactory.createDefaultTalon(Constants.kSerializerLeftRollerMasterId);
        mLeftRollerMaster.setInverted(false);
        mLeftRollerMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kLongCANTimeoutMs);
        mLeftRollerMaster.configVelocityMeasurementWindow(16, Constants.kLongCANTimeoutMs);


        mLeftRollerMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                10, Constants.kLongCANTimeoutMs);
        mLeftRollerMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mLeftRollerMaster.enableVoltageCompensation(true);

        // initialize encoders and set status frames
        TalonUtil.checkError(mRightRollerMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs), "Right Feeder Roller: Could not detect encoder: ");

        TalonUtil.checkError(mRightRollerMaster.config_kP(0, Constants.kFeederRollersKp, Constants.kLongCANTimeoutMs),
                "Right Feeder Roller: could not set kP: ");
        TalonUtil.checkError(mRightRollerMaster.config_kI(0, Constants.kFeederRollersKi, Constants.kLongCANTimeoutMs),
                "Right Feeder Roller: could not set kI: ");
        TalonUtil.checkError(mRightRollerMaster.config_kD(0, Constants.kFeederRollersKd, Constants.kLongCANTimeoutMs),
                "Right Feeder Roller: could not set kD: ");
        TalonUtil.checkError(mRightRollerMaster.config_kF(0, Constants.kFeederRollersKf, Constants.kLongCANTimeoutMs),
                "Right Feeder Roller: Could not set kF: ");
        TalonUtil.checkError(mRightRollerMaster.configAllowableClosedloopError(0, Constants.kFeederAllowableError),
                "Right Feeder Roller: Could not set closed loop allowable error");


        TalonUtil.checkError(mLeftRollerMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs), "Left Feeder Roller: Could not detect encoder: ");
        TalonUtil.checkError(mLeftRollerMaster.config_kP(0, Constants.kFeederRollersKp, Constants.kLongCANTimeoutMs),
                "Left Feeder Roller: could not set kP: ");
        TalonUtil.checkError(mLeftRollerMaster.config_kI(0, Constants.kFeederRollersKi, Constants.kLongCANTimeoutMs),
                "Left Feeder Roller: could not set kI: ");
        TalonUtil.checkError(mLeftRollerMaster.config_kD(0, Constants.kFeederRollersKd, Constants.kLongCANTimeoutMs),
                "Left Feeder Roller: could not set kD: ");
        TalonUtil.checkError(mLeftRollerMaster.config_kF(0, Constants.kFeederRollersKf, Constants.kLongCANTimeoutMs),
                "Left Feeder Roller: Could not set kF: ");
        TalonUtil.checkError(mLeftRollerMaster.configAllowableClosedloopError(0, Constants.kFeederAllowableError),
                "Left Feeder Roller: Could not set closed loop allowable error");

        mSkatePark = new Solenoid(Constants.kPCMId, Constants.kSkateParkSolenoidId);
        mIsSkateParkDeployed = true;
        setSkateParkDeployed(false);

        mChock = new Solenoid(Constants.kPCMId, Constants.kChockSolenoidId);
        mIsChockDeployed = true;
        setChockDeployed(false);

        mCanifier = Canifier.getInstance();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.right_roller_velocity = mRightRollerMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.left_roller_velocity = mLeftRollerMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.break_beam_triggered = mCanifier.isBreamBeamSensorTriggered();
        if (mPeriodicIO.break_beam_triggered) {
            mLastBreakBreakTriggerTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mSpinCycleMaster.set(ControlMode.PercentOutput, mPeriodicIO.spin_cycle_demand);
        mRightRollerMaster.set(ControlMode.Velocity, mPeriodicIO.right_roller_demand);
        mLeftRollerMaster.set(ControlMode.Velocity, mPeriodicIO.left_roller_demand);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Serializer.this) {
                    mIsChockDeployed = true;
                    mIsSkateParkDeployed = true;
                    setChockDeployed(false);
                    setSkateParkDeployed(false);
                    mWantedState = WantedState.IDLE;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Serializer.this) {
                    SystemState newState = mSystemState;
                    double timeInState = timestamp - mCurrentStateStartTime;

                    switch (mSystemState) {
                        case IDLE:
                            newState = handleIdle();
                            break;
                        case SERIALIZE:
                            newState = handleSerialize(timestamp);
                            break;
                        case FEED:
                            newState = handleFeed();
                            break;
                        case REBALANCING:
                            newState = handleRebalancing(timeInState);
                            break;
                        case REBALANCED:
                            newState = handleRebalanced();
                            break;
                        default:
                            System.out.println("unexpected serializer system state: " + mSystemState);
                            break;
                    }

                    if (newState != mSystemState) {
                        System.out.println(timestamp + ": Serializer changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        mCurrentStateStartTime = timestamp;
                        timeInState = 0.0;
                    }

                    switch (mSystemState) {
                        case IDLE:
                            setIdleStateDemands();
                            break;
                        case SERIALIZE:
                            setSerializeStateDemands();
                            break;
                        case FEED:
                            setFeedStateDemands();
                            break;
                        case REBALANCING:
                            setRebalancingStateDemands(timeInState);
                            break;
                        case REBALANCED:
                            setRebalancedStateDemands(timeInState);
                            break;
                        default:
                            System.out.println("Unexpected serializer system state: " + mSystemState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    private SystemState handleIdle() {
        switch (mWantedState) {
            case PREPARE_TO_SHOOT:
                return SystemState.REBALANCING;
            case SERIALIZE:
                return SystemState.SERIALIZE;
            case FEED:
                return SystemState.REBALANCING;
            case IDLE:
            default:
                if (mPeriodicIO.break_beam_triggered && !mSerializerCanceled) {
                    return SystemState.SERIALIZE;
                }
                return SystemState.IDLE;
        }
    }

    private SystemState handleSerialize(double timestamp) {
        switch (mWantedState) {
            case PREPARE_TO_SHOOT:
                if (!mSerializerCanceled && timestamp - mLastBreakBreakTriggerTime < kBeamBreakSerializeTime) {
                    return SystemState.SERIALIZE;
                }
                return SystemState.REBALANCING;
            case IDLE:
                if (!mSerializerCanceled && timestamp - mLastBreakBreakTriggerTime < kBeamBreakSerializeTime) {
                    return SystemState.SERIALIZE;
                }
                return SystemState.IDLE;
            case FEED:
                return SystemState.REBALANCING;
            case SERIALIZE:
            default:
                return SystemState.SERIALIZE;
        }
    }

    private SystemState handleFeed() {
        switch (mWantedState) {
            case PREPARE_TO_SHOOT:
                return SystemState.REBALANCING;
            case IDLE:
                return SystemState.IDLE;
            case SERIALIZE:
                return SystemState.SERIALIZE;
            case FEED:
            default:
                return SystemState.FEED;
        }
    }

    private SystemState handleRebalancing(double timeInState) {

        if (timeInState < kRebalancingTime && !mPeriodicIO.break_beam_triggered) {
            return SystemState.REBALANCING;
        }

        switch (mWantedState) {
            case PREPARE_TO_SHOOT:
            case IDLE:
                if (timeInState >= kRebalancingTime && !mPeriodicIO.break_beam_triggered) {
                    return SystemState.REBALANCED;
                } else if (mPeriodicIO.break_beam_triggered && !mSerializerCanceled) {
                    return SystemState.SERIALIZE;
                }
                return SystemState.REBALANCING;
            case SERIALIZE:
                return SystemState.SERIALIZE;
            case FEED:
                if (timeInState >= kRebalancingTime) {
                    return SystemState.REBALANCED;
                }
                return SystemState.REBALANCING;
            default:
                return SystemState.REBALANCING;
        }
    }

    private SystemState handleRebalanced() {

        switch (mWantedState) {
            case SERIALIZE:
                return SystemState.SERIALIZE;
            case FEED:
                return SystemState.FEED;
            case PREPARE_TO_SHOOT:
            case IDLE:
            default:
                if (mPeriodicIO.break_beam_triggered && !mSerializerCanceled) {
                    return SystemState.SERIALIZE;
                }
                return SystemState.REBALANCED;
        }
    }

    private void setIdleStateDemands() {
        if (!mStirOverride) {
            mPeriodicIO.spin_cycle_demand = 0.0;
        }

        mPeriodicIO.left_roller_demand = 0.0;
        mPeriodicIO.right_roller_demand = 0.0;
        setSkateParkDeployed(false);
        if (mSerializerCanceled) {
            setChockDeployed(false);
        } else {
            setChockDeployed(true);
        }
    }

    private void setSerializeStateDemands() {
        mPeriodicIO.spin_cycle_demand = kSpinCycleSerializeDemand;
        mPeriodicIO.left_roller_demand = 0.0;
        mPeriodicIO.right_roller_demand = 0.0;
        setSkateParkDeployed(false);
        if (mSerializerCanceled) {
            setChockDeployed(false);
        } else {
            setChockDeployed(true);
        }
    }

    public synchronized void setSpinCycleFeedSpeed(double demand) {
        mSpinCycleFeedDemand = demand;
    }

    private void setFeedStateDemands() {
        mPeriodicIO.spin_cycle_demand = mSpinCycleFeedDemand;
        mPeriodicIO.left_roller_demand = kRollerDemandFeed;
        mPeriodicIO.right_roller_demand = kRollerDemandFeed;
        setSkateParkDeployed(true);
        setChockDeployed(false);
    }

    private void setRebalancingStateDemands(double timeInState) {
        if (timeInState < kRebalancingTime) {
            mPeriodicIO.spin_cycle_demand = kSpinCycleRebalancingDemand;
        } else {
            mPeriodicIO.spin_cycle_demand = 0.0;
        }
        mPeriodicIO.left_roller_demand = 0.0;
        mPeriodicIO.right_roller_demand = 0.0;
        setSkateParkDeployed(false);
        setChockDeployed(false);
    }

    private void setRebalancedStateDemands(double timeInState) {
        if (!mStirOverride) {
            mPeriodicIO.spin_cycle_demand = 0.0;
        }
        mPeriodicIO.left_roller_demand = 0.0;
        mPeriodicIO.right_roller_demand = 0.0;
        setSkateParkDeployed(false);
        setChockDeployed(false);
    }

    private synchronized void setSkateParkDeployed(boolean should_deploy) {
        if (should_deploy != mIsSkateParkDeployed) {
            mSkatePark.set(should_deploy);
            mIsSkateParkDeployed = should_deploy;
        }
    }

    public static double rpmToNativeUnits(double rpm) {
        return rpm / 60.0 / 10.0 * Constants.kFeederRollersTicksPerRevolutions;
    }

    public static double nativeUnitsToRpm(double native_units) {
        return native_units * 60.0 * 10.0 / Constants.kFeederRollersTicksPerRevolutions;
    }

    private synchronized void setChockDeployed(boolean should_deploy) {
        if (should_deploy != mIsChockDeployed) {
            mChock.set(should_deploy);
            mIsChockDeployed = should_deploy;
        }
    }

    public synchronized void setOpenLoop(double demand) {
        mPeriodicIO.spin_cycle_demand = demand;
    }

    public synchronized void setSerializerCanceled(boolean cancel) {
        mSerializerCanceled = cancel;
    }

    public synchronized void setStirOverriding(boolean override) {
        mStirOverride = override;
    }


    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    @Override
    public synchronized void stop() {
        mSpinCycleMaster.set(ControlMode.PercentOutput, 0.0);
        mRightRollerMaster.set(ControlMode.PercentOutput, 0.0);
        mLeftRollerMaster.set(ControlMode.PercentOutput, 0.0);
        mIsChockDeployed = true;
        mIsSkateParkDeployed = true;
        setChockDeployed(false);
        setSkateParkDeployed(false);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Serializer Spin Cycle Demand", mPeriodicIO.spin_cycle_demand);
        SmartDashboard.putNumber("Serializer Right Roller Demand RPM", nativeUnitsToRpm(mPeriodicIO.right_roller_demand));
        SmartDashboard.putNumber("Serializer Left Roller Demand RPM", nativeUnitsToRpm(mPeriodicIO.left_roller_demand));

        SmartDashboard.putNumber("Serializer Right Roller Speed RPM", nativeUnitsToRpm(mPeriodicIO.right_roller_demand));
        SmartDashboard.putNumber("Serializer Left Roller Speed RPM", nativeUnitsToRpm(mPeriodicIO.left_roller_demand));

        SmartDashboard.putBoolean("Is Break Beam Sensor Triggered", mPeriodicIO.break_beam_triggered);

        SmartDashboard.putBoolean("Serializer Cancelled", mSerializerCanceled);
    }
}