package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team254.frc2020.Constants;
import com.team254.frc2020.RobotState;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonFXChecker;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveOutput;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

public class Drive extends Subsystem {

    private static Drive mInstance;
    private AtomicBoolean mIsPTOEngaged = new AtomicBoolean(false);

    // hardware
    private final TalonFX mLeftMaster1, mRightMaster1, mLeftMaster2, mRightMaster2, mLeftMaster3, mRightMaster3;
    private final Solenoid mShifter;
    private final Solenoid mPTO;
    private final Solenoid mBrake;
    private final Solenoid mDeploy;
    private final Encoder mLeftEncoder, mRightEncoder;
    private final DriveWithPTOSide mLeftSide;
    private final DriveWithPTOSide mRightSide;

    // control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;

    // hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();

    private DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private int kHighGearPIDSlot = 0;
    private int kLowGearPIDSlot = 1;
    private int kPositionPID = 2;

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private void configureTalon(TalonFX talon, boolean left, boolean main_encoder_talon) {
        // general
        talon.setInverted(!left);
        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);

        // pid
        TalonUtil.checkError(talon.config_kP(kHighGearPIDSlot, Constants.kDriveHighGearKp, Constants.kLongCANTimeoutMs), "Could not set high gear kp");
        TalonUtil.checkError(talon.config_kI(kHighGearPIDSlot, Constants.kDriveHighGearKi, Constants.kLongCANTimeoutMs), "Could not set high gear ki");
        TalonUtil.checkError(talon.config_kD(kHighGearPIDSlot, Constants.kDriveHighGearKd, Constants.kLongCANTimeoutMs), "Could not set high gear kd");
        TalonUtil.checkError(talon.config_kF(kHighGearPIDSlot, Constants.kDriveHighGearKf, Constants.kLongCANTimeoutMs), "Could not set high gear kf");

        TalonUtil.checkError(talon.config_kP(kLowGearPIDSlot, Constants.kDriveLowGearKp, Constants.kLongCANTimeoutMs), "Could not set low gear kp");
        TalonUtil.checkError(talon.config_kI(kLowGearPIDSlot, Constants.kDriveLowGearKi, Constants.kLongCANTimeoutMs), "Could not set low gear ki");
        TalonUtil.checkError(talon.config_kD(kLowGearPIDSlot, Constants.kDriveLowGearKd, Constants.kLongCANTimeoutMs), "Could not set low gear kd");
        TalonUtil.checkError(talon.config_kF(kLowGearPIDSlot, Constants.kDriveLowGearKf, Constants.kLongCANTimeoutMs), "Could not set low gear kf");

        TalonUtil.checkError(talon.config_kP(kPositionPID, Constants.kDrivePositionKp, Constants.kLongCANTimeoutMs), "Could not set low gear kp");
        TalonUtil.checkError(talon.config_kI(kPositionPID, Constants.kDrivePositionKi, Constants.kLongCANTimeoutMs), "Could not set low gear ki");
        TalonUtil.checkError(talon.config_kD(kPositionPID, Constants.kDrivePositionKd, Constants.kLongCANTimeoutMs), "Could not set low gear kd");
        TalonUtil.checkError(talon.config_kF(kPositionPID, Constants.kDrivePositionKf, Constants.kLongCANTimeoutMs), "Could not set low gear kf");

        TalonUtil.checkError(talon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0.2), Constants.kLongCANTimeoutMs), "Could not set stator drive current limits");

        TalonUtil.checkError(talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs), "could not config drive velocity measurement period");
        TalonUtil.checkError(talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs), "could not config drive velocity measurement window");


        // voltage comp
        TalonUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "could not config drive voltage comp saturation");
        talon.enableVoltageCompensation(true);

        if (main_encoder_talon) {
            // status frames (maybe set for characterization?)
            TalonUtil.checkError(talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, Constants.kLongCANTimeoutMs), "could not set drive feedback frame");
            // TalonUtil.checkError(talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10, Constants.kLongCANTimeoutMs), "could not set drive voltage frame");

            // velocity measurement


        }
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftMaster1 = TalonFXFactory.createDefaultTalon(Constants.kLeftDriveMaster1Id);
        configureTalon(mLeftMaster1, true, false);

        mLeftMaster2 = TalonFXFactory.createDefaultTalon(Constants.kLeftDriveMaster2Id);
        configureTalon(mLeftMaster2, true, true);

        mLeftMaster3 = TalonFXFactory.createDefaultTalon(Constants.kLeftDriveMaster3Id);
        configureTalon(mLeftMaster3, true, false);

        mRightMaster1 = TalonFXFactory.createDefaultTalon(Constants.kRightDriveMaster1Id);
        configureTalon(mRightMaster1, false, false);

        mRightMaster2 = TalonFXFactory.createDefaultTalon(Constants.kRightDriveMaster2Id);
        configureTalon(mRightMaster2, false, true);

        mRightMaster3 = TalonFXFactory.createDefaultTalon(Constants.kRightDriveMaster3Id);
        configureTalon(mRightMaster3, false, false);

        // left and right master 1 are PTO talons, 2 in middle, 3 in back
        mLeftSide = new DriveWithPTOSide(mLeftMaster1, mLeftMaster2, mLeftMaster3);
        mRightSide = new DriveWithPTOSide(mRightMaster1, mRightMaster2, mRightMaster3);

        mShifter = new Solenoid(Constants.kPCMId, Constants.kShifterSolenoidId);

        mPTO = new Solenoid(Constants.kPCMId, Constants.kPTOSolenoidId);

        mPigeon = new PigeonIMU(Constants.kPigeonIMUId);
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 10, 10);

        // force a solenoid message
        mIsHighGear = false;
        setHighGear(true);

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();

        mLeftEncoder = new Encoder(Constants.kLeftDriveEncoderA, Constants.kLeftDriveEncoderB, false);
        mRightEncoder = new Encoder(Constants.kRightDriveEncoderA, Constants.kRightDriveEncoderB, true);

        mLeftEncoder.setReverseDirection(true);
        mRightEncoder.setReverseDirection(false);

        mBrake = new Solenoid(Constants.kPCMId, Constants.kBrakeSolenoidId);
        mDeploy = new Solenoid(Constants.kPCMId, Constants.kDeploySolenoidId);

        resetEncoders();
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public static class PeriodicIO {
        // real outputs
        public double left_demand;
        public double right_demand;

        // INPUTS
        public double timestamp;
        public double left_voltage;
        public double right_voltage;
        public int left_position_ticks; // using us digital encoder
        public int right_position_ticks; // us digital encoder
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms; // using Talon FX
        public int right_velocity_ticks_per_100ms; // Talon FX
        public double left_velocity_in_per_sec;
        public double right_velocity_in_per_sec;
        public Rotation2d gyro_heading = Rotation2d.identity();

        // OUTPUTS
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public Pose2d error = Pose2d.identity();
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }


    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.left_voltage = mLeftSide.getPrimaryDriveTalonFX().getMotorOutputVoltage();
        mPeriodicIO.right_voltage = mRightSide.getPrimaryDriveTalonFX().getMotorOutputVoltage();

        mPeriodicIO.left_position_ticks = mLeftEncoder.get();
        mPeriodicIO.right_position_ticks = mRightEncoder.get();

        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

        mPeriodicIO.left_distance = rotationsToInches(mPeriodicIO.left_position_ticks * getRotationsPerTickDistance());
        mPeriodicIO.right_distance = rotationsToInches(mPeriodicIO.right_position_ticks * getRotationsPerTickDistance());

        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftSide.getPrimaryDriveTalonFX().getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightSide.getPrimaryDriveTalonFX().getSelectedSensorVelocity(0);

        mPeriodicIO.left_velocity_in_per_sec = getLeftLinearVelocity();
        mPeriodicIO.right_velocity_in_per_sec = getRightLinearVelocity();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftSide.getDriveTalons().forEach(t -> t.set(ControlMode.PercentOutput, mPeriodicIO.left_demand));
            mRightSide.getDriveTalons().forEach(t -> t.set(ControlMode.PercentOutput, mPeriodicIO.right_demand));
        } else if (mDriveControlState == DriveControlState.VELOCITY || mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            double kd = isHighGear() ? Constants.kDriveHighGearKd : Constants.kDriveLowGearKd;
            mLeftSide.getDriveTalons().forEach(t -> t.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + kd * mPeriodicIO.left_accel / 1023.0));
            mRightSide.getDriveTalons().forEach(t -> t.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + kd * mPeriodicIO.right_accel / 1023.0));
        }
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    stop();
                    setBrakeMode(true);
                    startLogging();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case VELOCITY:
                            break;
                        case PATH_FOLLOWING:
                            updatePathFollower();
                            break;
                        default:
                            System.out.println("unexpected drive control state: " + mDriveControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
            }
        });
    }

    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesToRadians(double inches) {
        return inches * 2.0 / Constants.kDriveWheelDiameterInches;
    }

    public static double radiansToInches(double radians) {
        return radians / 2.0 * Constants.kDriveWheelDiameterInches;
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    /**
     * @param rad_s of the output
     * @return ticks per 100 ms of the talonfx encoder
     */
    private double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) / getRotationsPerTickVelocity() / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    /**
     * Configure talons for velocity control during teleop
     */
    public synchronized void setVelocity(DriveOutput output) {
        if (mDriveControlState != DriveControlState.VELOCITY) {
            setBrakeMode(true);
            System.out.println("switching to velocity");
            mDriveControlState = DriveControlState.VELOCITY;
            configureTalonPIDSlot();
        }

        mPeriodicIO.left_demand = radiansPerSecondToTicksPer100ms(output.left_velocity);
        mPeriodicIO.right_demand = radiansPerSecondToTicksPer100ms(output.right_velocity);
        mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
        mPeriodicIO.left_feedforward = Util.epsilonEquals(mPeriodicIO.left_demand, 0.0) ? 0.0 :
                output.left_feedforward_voltage / 12.0;
        mPeriodicIO.right_feedforward = Util.epsilonEquals(mPeriodicIO.right_demand, 0.0) ? 0.0 :
                output.right_feedforward_voltage / 12.0;
    }

    /**
     * Configure talons for following via the ramsete controller
     */
    public synchronized void setRamseteVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            configureTalonPIDSlot();
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveOutput output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setRamseteVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setRamseteVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void configureTalonPIDSlot() {
        int desired_slot_idx = isHighGear() ? kHighGearPIDSlot : kLowGearPIDSlot;

        mLeftSide.getDriveTalons().forEach(t -> t.selectProfileSlot(desired_slot_idx, 0));
        mRightSide.getDriveTalons().forEach(t -> t.selectProfileSlot(desired_slot_idx, 0));
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            // Plumbed default high.
            mShifter.set(!wantsHighGear);
            configureTalonPIDSlot();
        }
    }

    public synchronized void setPTOEngaged(boolean wantsEngage) {
        mPTO.set(wantsEngage);
        mIsPTOEngaged.set(wantsEngage);
        mLeftSide.setPTOEngaged(wantsEngage);
        mRightSide.setPTOEngaged(wantsEngage);

        if (!wantsEngage) {
            // Retrigger brake mode.
            boolean cached = mIsBrakeMode;
            mIsBrakeMode = !mIsBrakeMode;
            setBrakeMode(cached);
        } else {
            mLeftSide.getPTOMotors().forEach(
                    t -> t.setNeutralMode(NeutralMode.Brake));
            mRightSide.getPTOMotors().forEach(
                    t -> t.setNeutralMode(NeutralMode.Brake));
        }
    }

    public synchronized void setDeploy(boolean deploy) {
        mDeploy.set(deploy);
    }

    public synchronized boolean getDeploy() {
        return mDeploy.get();
    }


    public synchronized void setBrakeEngaged(boolean wantsEngage) {
        mBrake.set(!wantsEngage);
    }

    public synchronized boolean getBrake() {
        return !mBrake.get();
    }

    public synchronized void configPTOPID(boolean wantsPosition) {
        if (wantsPosition) {
            mLeftSide.getPTOMotors().forEach(
                    t -> t.selectProfileSlot(kPositionPID, 0));
            mRightSide.getPTOMotors().forEach(
                    t -> t.selectProfileSlot(kPositionPID, 0));
        } else {
            final int desired_slot_idx = isHighGear() ? kHighGearPIDSlot : kLowGearPIDSlot;
            mLeftSide.getPTOMotors().forEach(
                    t -> t.selectProfileSlot(desired_slot_idx, 0));
            mRightSide.getPTOMotors().forEach(
                    t -> t.selectProfileSlot(desired_slot_idx, 0));
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            NeutralMode mode = shouldEnable ? NeutralMode.Brake : NeutralMode.Coast;

            mLeftSide.getDriveTalons().forEach(t -> t.setNeutralMode(mode));
            mRightSide.getDriveTalons().forEach(t -> t.setNeutralMode(mode));
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    public synchronized void resetEncoders() {

        mLeftEncoder.reset();
        mRightEncoder.reset();

        mLeftSide.getAllMotors().forEach(t -> t.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs));
        mRightSide.getAllMotors().forEach(t -> t.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs));

        mPeriodicIO = new PeriodicIO();
    }

    public double getLeftEncoderDistance() {
        return mPeriodicIO.left_distance;
    }

    public double getRightEncoderDistance() {
        return mPeriodicIO.right_distance;
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10 * getRotationsPerTickVelocity());
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10 * getRotationsPerTickVelocity());
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity())) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public double getLeftOutputVoltage() {
        return mPeriodicIO.left_voltage;
    }

    public double getRightOutputVoltage() {
        return mPeriodicIO.right_voltage;
    }

    public double getAverageOutputVoltageMagnitude() {
        return (Math.abs(getLeftOutputVoltage()) + Math.abs(getRightOutputVoltage())) / 2.0;
    }

    /**
     * @return conversion factor where ticks * getEncoderTicksPerRotation() = wheel rotations
     */
    public double getRotationsPerTickVelocity() { // talonfx
        return isHighGear() ? Constants.kDriveRotationsPerTickHighGear : Constants.kDriveRotationsPerTickLowGear;
    }

    public double getRotationsPerTickDistance() { // us digital encoders
        return 1.0 / Constants.kDriveEncoderPPR;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control,
        VELOCITY, // velocity control
        PATH_FOLLOWING
    }

    public enum ShifterState {
        FORCE_LOW_GEAR, FORCE_HIGH_GEAR
    }

    public synchronized void setPTOMotorsOpenLoop(double signal, double feedforward) {
        mLeftSide.getPTOMotors().forEach(t -> t.set(ControlMode.PercentOutput, signal,
                DemandType.ArbitraryFeedForward, feedforward));
        mRightSide.getPTOMotors().forEach(t -> t.set(ControlMode.PercentOutput, signal,
                DemandType.ArbitraryFeedForward, feedforward));
    }

    public synchronized void zeroPTOMotors() {
        mLeftSide.getPTOMotors().forEach(t -> t.setSelectedSensorPosition(0));
        mRightSide.getPTOMotors().forEach(t -> t.setSelectedSensorPosition(0));
    }

    public synchronized void configPTOCurrentLimits(double current) {
        mLeftSide.getPTOMotors().forEach(t -> t.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                true, current, current, 0.2), 0));
        mRightSide.getPTOMotors().forEach(t -> t.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(
                true, current, current, 0.2), 0));
    }

    public synchronized double getPTOPosition() {
        return mRightSide.getPTOMotors().iterator().next().getSelectedSensorPosition();
    }

    public synchronized void setPTOMotorsPosition(double position) {
        mLeftSide.getPTOMotors().forEach(t -> t.set(ControlMode.Position, position));
        mRightSide.getPTOMotors().forEach(t -> t.set(ControlMode.Position, position));
    }


    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setHighGear(true);

        // Trigger write to TalonFXs.
        stop();
        writePeriodicOutputs();

        setBrakeMode(false);

        boolean leftSide = TalonFXChecker.checkMotors(this,
                new ArrayList<>() {
                    {
                        add(new MotorChecker.MotorConfig<>("left_master", mLeftMaster1));
                        add(new MotorChecker.MotorConfig<>("left_master_2", mLeftMaster2));
                        add(new MotorChecker.MotorConfig<>("left_master_3", mLeftMaster3));
                    }
                }, new MotorChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 5;
                        mRPMFloor = 90;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 200;
                        mRPMSupplier = mLeftEncoder::getRate;
                    }
                });
        boolean rightSide = TalonFXChecker.checkMotors(this,
                new ArrayList<>() {
                    {
                        add(new MotorChecker.MotorConfig<>("right_master", mRightMaster1));
                        add(new MotorChecker.MotorConfig<>("right_master_2", mRightMaster2));
                        add(new MotorChecker.MotorConfig<>("right_master_3", mRightMaster3));

                    }
                }, new MotorChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 5;
                        mRPMFloor = 90;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 200;
                        mRPMSupplier = mRightEncoder::getRate;
                    }
                });

        return leftSide && rightSide;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        SmartDashboard.putNumber("Left Drive 1 Current", mLeftMaster1.getStatorCurrent());
        SmartDashboard.putNumber("Left Drive 2 Current", mLeftMaster2.getStatorCurrent());
        SmartDashboard.putNumber("Left Drive 3 Current", mLeftMaster3.getStatorCurrent());
        SmartDashboard.putNumber("Right Drive 1 Current", mRightMaster1.getStatorCurrent());
        SmartDashboard.putNumber("Right Drive 2 Current", mRightMaster2.getStatorCurrent());
        SmartDashboard.putNumber("Right Drive 3 Current", mRightMaster3.getStatorCurrent());

        SmartDashboard.putNumber("Left Drive Demand", mPeriodicIO.left_demand);
        SmartDashboard.putNumber("Right Drive Demand", mPeriodicIO.right_demand);
        SmartDashboard.putNumber("Left Drive Feedforward", mPeriodicIO.left_feedforward);
        SmartDashboard.putNumber("Right Drive Feedforward", mPeriodicIO.right_feedforward);

        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    public static class DriveWithPTOSide {
        private final TalonFX driveOnlyA, driveOnlyB, driveAndClimb;
        private Set<TalonFX> onlyDrive = new HashSet<>();
        private Set<TalonFX> allMotors = new HashSet<>();
        private Set<TalonFX> withPTO = new HashSet<>();
        private Set<TalonFX> emptySet = new HashSet<>();

        private AtomicBoolean mPtoEngaged = new AtomicBoolean(false);

        public DriveWithPTOSide(TalonFX driveAndClimb, TalonFX driveOnlyA, TalonFX driveOnlyB) {
            this.driveOnlyA = driveOnlyA;
            this.driveOnlyB = driveOnlyB;
            this.driveAndClimb = driveAndClimb;

            allMotors.add(driveAndClimb);
            allMotors.add(driveOnlyA);
            allMotors.add(driveOnlyB);

            onlyDrive.add(driveOnlyA);
            onlyDrive.add(driveOnlyB);

            withPTO.add(driveAndClimb);
        }

        public TalonFX getPrimaryDriveTalonFX() {
            return driveOnlyA;
        }

        public void setPTOEngaged(boolean engaged) {
            mPtoEngaged.set(engaged);
        }

        public Set<TalonFX> getDriveTalons() {
            if (mPtoEngaged.get()) {
                return onlyDrive;
            } else {
                return allMotors;
            }
        }

        public Set<TalonFX> getPTOMotors() {
            if (mPtoEngaged.get()) {
                return withPTO;
            } else {
                return emptySet;
            }
        }

        public Set<TalonFX> getAllMotors() {
            return allMotors;
        }
    }
}