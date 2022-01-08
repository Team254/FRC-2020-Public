package com.team254.frc2020.subsystems;

import com.team254.frc2020.Constants;
import com.team254.frc2020.RobotState;
import com.team254.frc2020.limelight.LimelightManager;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.frc2020.states.TimedLEDState;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.ShootingParameters;
import com.team254.lib.util.ShootingParameters.BallQuality;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class Superstructure extends Subsystem {
    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Turret mTurret = Turret.getInstance();
    private Hood mHood = Hood.getInstance();
    private LED mLED = LED.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private Serializer mSerializer = Serializer.getInstance();
    private LimelightManager mLLManager = LimelightManager.getInstance();

    private RobotState mRobotState = RobotState.getInstance();

    private Superstructure() {
        if (Constants.kIsHoodTuning) {
            SmartDashboard.putNumber("HoodAngleToSet", 50.0);
            SmartDashboard.putNumber("ShooterRPMToSet", 4500.0);
        }
    }

    private double kCoarseHoodMapBias = -1.0;
    private double kFineHoodMapBias = 0.0;

    public enum WantedState {
        IDLE,
        AIM,
        SHOOT,
        MOVE_TO_ZERO
    }

    public enum SystemState {
        IDLE,
        AIMING,
        SHOOT,
        MOVE_TO_ZERO
    }

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;
    private double mLastShootingParamsPrintTime = 0.0;

    private boolean mOverrideLimelightLEDs = false;

    private Optional<Double> mTurretHint = Optional.empty();
    private Optional<Double> mTurretJogDelta = Optional.empty();

    private ShootingParameters mShootingParameters = Constants.kCoarseShootingParams;
    private BallQuality mBallQuality = BallQuality.MEDIUM_BALL;

    private double mTurretFeedforwardVFromVision = 0.0;

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mWantedState = WantedState.IDLE;
                    mOverrideLimelightLEDs = false;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    SystemState newState = mSystemState;

                    switch (mSystemState) {
                        case IDLE:
                            newState = handleIdle(mWantedState);
                            break;
                        case AIMING:
                            newState = handleAiming(mWantedState);
                            break;
                        case SHOOT:
                            newState = handleShoot(mWantedState);
                            break;
                        case MOVE_TO_ZERO:
                            newState = handleMoveToZero(mWantedState);
                            break;
                        default:
                            System.out.println("unexpected superstructure system state: " + mSystemState);
                            break;
                    }

                    if (newState != mSystemState) {
                        System.out.println(timestamp + ": Superstructure changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                    }

                    switch (mSystemState) {
                        case IDLE:
                            writeIdleDesiredState(timestamp);
                            break;
                        case AIMING:
                            writeAimingDesiredState(timestamp);
                            break;
                        case SHOOT:
                            writeShootDesiredState(timestamp);
                            break;
                        case MOVE_TO_ZERO:
                            writeMoveToZeroDesiredState();
                            break;
                        default:
                            System.out.println("Unexpected superstructure system state: " + mSystemState);
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

    private SystemState handleIdle(WantedState wantedState) {
        switch (wantedState) {
            case AIM:
                return SystemState.AIMING;
            case MOVE_TO_ZERO:
                mHood.setHoming();
                return SystemState.MOVE_TO_ZERO;
            case IDLE:
            case SHOOT:
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleAiming(WantedState wantedState) {
        switch (wantedState) {
            case MOVE_TO_ZERO:
                mHood.setHoming();
                return SystemState.MOVE_TO_ZERO;
            case IDLE:
                return SystemState.IDLE;
            case SHOOT:
                SmartDashboard.putBoolean("ShooterAtSetpoint", mShootingParameters.isShooterAtSetpoint(mShooter.getAverageRPM(), mShooter.getDemandRPM()));
                SmartDashboard.putBoolean("TurretAtSetpoint", mShootingParameters.isTurretAtSetpoint(mTurret.getAngle(), mTurret.getSetpointHomed()));
                SmartDashboard.putBoolean("HoodAtSetpoint", mShootingParameters.isHoodAtSetpoint(mHood.getAngle(), mHood.getSetpointHomed()));

                if (mShootingParameters.isShooterAtSetpoint(mShooter.getAverageRPM(), mShooter.getDemandRPM()) &&
                        visionHasTarget() && mShootingParameters.isTurretAtSetpoint(mTurret.getAngle(), mTurret.getSetpointHomed()) &&
                        mShootingParameters.isHoodAtSetpoint(mHood.getAngle(), mHood.getSetpointHomed())) {
                    return SystemState.SHOOT;
                }
            default:
                return SystemState.AIMING;
        }
    }

    private SystemState handleShoot(WantedState wantedState) {
        switch (wantedState) {
            case MOVE_TO_ZERO:
                mHood.setHoming();
                return SystemState.MOVE_TO_ZERO;
            case IDLE:
                return SystemState.IDLE;
            case AIM:
                return SystemState.AIMING;
            case SHOOT:
            default:
                return SystemState.SHOOT;
        }
    }

    private SystemState handleMoveToZero(WantedState wantedState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case AIM:
                return SystemState.AIMING;
            case SHOOT:
                return SystemState.AIMING;
            case MOVE_TO_ZERO:
            default:
                return SystemState.MOVE_TO_ZERO;
        }
    }

    private void writeIdleDesiredState(double timestamp) {
        if (!mOverrideLimelightLEDs) {
            mLLManager.getTurretLimelight().setLed(Limelight.LedMode.OFF);
        }

        mSerializer.setWantedState(Serializer.WantedState.IDLE);

        if (mTurretHint.isPresent()) {
            mTurret.setSetpointPositionPID(getTurretSetpointFromFieldRelativeGoal(timestamp, mTurretHint.get()));
        } else if (mTurretJogDelta.isPresent()) {
            mTurret.setSetpointPositionPID(mTurret.getAngle() + mTurretJogDelta.get());
        } else {
            mTurret.setSetpointPositionPID(mTurret.getAngle());
        }

        mHood.setSetpointPositionPID(mHood.getAngle());
        mShooter.setOpenLoop(0.0);
        mLED.setSuperstructureLEDState(TimedLEDState.StaticLEDState.kStaticOff);
    }

    private void writeAimingDesiredState(double timestamp) {
        mSerializer.setWantedState(Serializer.WantedState.PREPARE_TO_SHOOT);
        if (!mOverrideLimelightLEDs) {
            mLLManager.getTurretLimelight().setLed(Limelight.LedMode.PIPELINE);
        }

        double visionAngle = getTurretSetpointFromVision(timestamp);
        double angleToSet = mTurret.getAngle();
        double ffToSet = 0;
        if (visionHasTarget()) {
            angleToSet = visionAngle;
            ffToSet = getTurretFeedforwardVFromVision();
        } else if (mTurretHint.isPresent()) {
            angleToSet = getTurretSetpointFromFieldRelativeGoal(timestamp, mTurretHint.get());
        } else if (mTurretJogDelta.isPresent()) {
            mTurret.setSetpointPositionPID(mTurret.getAngle() + mTurretJogDelta.get());
        } else {
            angleToSet = getTurretSetpointFromFieldRelativeGoal(timestamp, 0.0);
        }
        mTurret.setSetpointPositionPID(angleToSet, ffToSet);

        if (Constants.kIsHoodTuning) {
            mHood.setSetpointPositionPID(SmartDashboard.getNumber("HoodAngleToSet", 50.0));
            mShooter.setRPM(SmartDashboard.getNumber("ShooterRPMToSet", 4700.0));
        } else {
            if (mLatestAimingParameters.isPresent()) {
                mHood.setSetpointPositionPID(
                        mShootingParameters.getHoodMap(mBallQuality).getInterpolated(new InterpolatingDouble(mLatestAimingParameters.get().getRange())).value);
                mShooter.setRPM(
                        mShootingParameters.getShooterRPMMap().getInterpolated(new InterpolatingDouble(mLatestAimingParameters.get().getRange())).value);
            } else {
                mShooter.setRPM(Constants.kDefaultShooterRPM);
            }
        }
        mLED.setSuperstructureLEDState(TimedLEDState.BlinkingLEDState.kBlinkingAiming);
    }

    private void writeShootDesiredState(double timestamp) {
        if (!mOverrideLimelightLEDs) {
            mLLManager.getTurretLimelight().setLed(Limelight.LedMode.PIPELINE);
        }
        mSerializer.setSpinCycleFeedSpeed(mShootingParameters.getSpinCycleSetpoint());
        mSerializer.setWantedState(Serializer.WantedState.FEED);

        mTurret.setSetpointPositionPID(getTurretSetpointFromVision(timestamp), getTurretFeedforwardVFromVision());

        double hoodAngle = Double.NaN;
        double range = Double.NaN;
        double shooterRpm = Constants.kDefaultShooterRPM;

        if (mLatestAimingParameters.isPresent()) {
            range = mLatestAimingParameters.get().getRange();
        }

        if (Constants.kIsHoodTuning) {
            hoodAngle = SmartDashboard.getNumber("HoodAngleToSet", 50.0);
            shooterRpm = SmartDashboard.getNumber("ShooterRPMToSet", 4500.0);
        } else {
            if (mLatestAimingParameters.isPresent()) {
                hoodAngle = mShootingParameters.getHoodMap(mBallQuality).getInterpolated(new InterpolatingDouble(range)).value;
                shooterRpm = mShootingParameters.getShooterRPMMap().getInterpolated(new InterpolatingDouble(range)).value;
            } else {
                hoodAngle = mHood.getAngle();
                range = -1;
                System.out.println("Lost target while shooting! Maintaining: " + hoodAngle);
            }
        }
        mHood.setSetpointPositionPID(hoodAngle);

        mShooter.setRPM(shooterRpm);

        if ((timestamp - mLastShootingParamsPrintTime) > 0.5) {
            mLastShootingParamsPrintTime = timestamp;
            System.out.println("Making shot, range: " + range + " hood: " + mHood.getAngle() + " rpm: " + mShooter.getAverageRPM()
                    + " output voltage: " + mShooter.getAverageOutputVoltage()
                    + " supply current: " + mShooter.getAverageSupplyCurrent()
                    + " stator current: " + mShooter.getAverageStatorCurrent());
        }
        mLED.setSuperstructureLEDState(TimedLEDState.BlinkingLEDState.kBlinkingShooting);
    }

    private void writeMoveToZeroDesiredState() {
        if (!mOverrideLimelightLEDs) {
            mLLManager.getTurretLimelight().setLed(Limelight.LedMode.OFF);
        }
        mSerializer.setWantedState(Serializer.WantedState.IDLE);
        mTurret.setSetpointPositionPID(Constants.kTurretConstants.kHomePosition);
        mHood.setSetpointPositionPID(Constants.kHoodConstants.kHomePosition);
        mShooter.setOpenLoop(0.0);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void writePeriodicOutputs() {}

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized WantedState getWantedState() {
        return mWantedState;
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public boolean visionHasTarget() {
        return mHasTarget;
    }

    public void setOverrideLimelightLEDs(boolean should_override) {
        mOverrideLimelightLEDs = should_override;
    }

    public synchronized double getTurretSetpointFromVision(double timestamp) {
        mLatestAimingParameters = mRobotState.getAimingParameters(-1, Constants.kMaxGoalTrackAge, mShootingParameters.getVisionTargetToGoalOffset());
        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

            // if (Constants.kIsHoodTuning) {
                SmartDashboard.putNumber("Range To Target", mLatestAimingParameters.get().getRange());
            // }

            // Don't aim if not in min distance
            if (mEnforceAutoAimMinDistance && mLatestAimingParameters.get().getRange() > mAutoAimMinDistance) {
                return mTurret.getAngle();
            }

            Rotation2d turret_error = mRobotState.getVehicleToTurret(timestamp).inverse()
                    .transformBy(mRobotState.getFieldToVehicle(timestamp).inverse())
                    .transformBy(mLatestAimingParameters.get().getFieldToGoal()).getTranslation().direction();

            double turret_setpoint = mTurret.getAngle() + turret_error.getDegrees();
            Twist2d velocity = mRobotState.getMeasuredVelocity();
            // Angular velocity component from tangential robot motion about the goal.
            double tangential_component = mLatestAimingParameters.get().getRobotToGoalRotation().sin() * velocity.dx / mLatestAimingParameters.get().getRange();
            double angular_component = Units.radians_to_degrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local frame.
            mTurretFeedforwardVFromVision = -(angular_component + tangential_component);

            mHasTarget = true;

            mOnTarget = Util.epsilonEquals(turret_error.getDegrees(), 0.0, 3.0);

            return Util.limitTurret(turret_setpoint);
        } else {
            mHasTarget = false;
            mOnTarget = false;

            return mTurret.getAngle();
        }
    }

    /**
     * @param field_relative_goal in degrees
     */
    public synchronized double getTurretSetpointFromFieldRelativeGoal(double timestamp, double field_relative_goal) {
        Rotation2d turret_error = mRobotState.getFieldToVehicle(timestamp)
                .transformBy(mRobotState.getVehicleToTurret(timestamp)).getRotation().inverse()
                .rotateBy(Rotation2d.fromDegrees(field_relative_goal));
        double turret_setpoint = mTurret.getAngle() + turret_error.getDegrees();

        return Util.limitTurret(turret_setpoint);
    }

    public synchronized double getFieldRelativeTurretSetpointFromRobotGoal(double timestamp,
                                                                           double robot_relative_goal) {
        return mRobotState.getFieldToVehicle(timestamp).getRotation().rotateBy(
                Rotation2d.fromDegrees(robot_relative_goal)).getDegrees();
    }

    /**
     * pre condition: getTurretSetpointFromVision() is called
     *
     * @return turret feedforward voltage
     */
    public synchronized double getTurretFeedforwardVFromVision() {
        return mTurretFeedforwardVFromVision;
    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTurretFeedforwardVFromVision = 0.0;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    public synchronized boolean isOnTarget() {
        return mOnTarget;
    }

    public synchronized void setTurretHint(double hint) {
        mTurretHint = Optional.of(hint);
    }

    public synchronized void setTurretHintRobotRelative(double timestamp, double hint) {
        mTurretHint = Optional.of(getFieldRelativeTurretSetpointFromRobotGoal(timestamp, hint));
    }

    public synchronized void resetTurretHint() {
        mTurretHint = Optional.empty();
    }

    public synchronized void setTurretJog(double jog_delta) {
        mTurretJogDelta = Optional.of(jog_delta);
    }

    public synchronized void resetTurretJog() {
        mTurretJogDelta = Optional.empty();
    }

    public synchronized void setShootingParams(ShootingParameters shootingParameters) {
        if (shootingParameters != null) {
            mShootingParameters = shootingParameters;
        } else {
            DriverStation.reportError("Trying to set null Shooting Params in Superstructure", true);
        }
    }

    public synchronized void setBallQuality(BallQuality ballQuality) {
        mBallQuality = ballQuality;
    }
}