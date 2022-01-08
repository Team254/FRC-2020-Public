package com.team254.frc2020;

import com.team254.frc2020.auto.AutoModeExecutor;
import com.team254.frc2020.auto.modes.AutoModeBase;
import com.team254.frc2020.controlboard.CardinalDirection;
import com.team254.frc2020.controlboard.ControlBoard;
import com.team254.frc2020.controlboard.IControlBoard;
import com.team254.frc2020.limelight.LimelightManager;
import com.team254.frc2020.limelight.constants.LimelightConstantsFactory;
import com.team254.frc2020.loops.Looper;
import com.team254.frc2020.paths.TrajectoryGenerator;
import com.team254.frc2020.statemachines.ClimbingStateMachine;
import com.team254.frc2020.subsystems.*;
import com.team254.frc2020.states.TimedLEDState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.*;
import com.team254.lib.util.ShootingParameters.BallQuality;
import com.team254.lib.wpilib.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    // subsystems
    private final Drive mDrive = Drive.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final Limelight mLimelight = new Limelight(
            LimelightConstantsFactory.getConstantsForThisRobot(),
            Constants.kLowRes1xZoom);
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Serializer mSerializer = Serializer.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Canifier mCanifier = Canifier.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();
    private final WOF mWOF = WOF.getInstance();
    private final LED mLED = LED.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();

    private DelayedBoolean mShouldNotShoot;
    private DelayedBoolean mShouldNotAim;
    private ClimbingStateMachine mClimbingStateMachine = new ClimbingStateMachine();
    private LatchedBoolean mHangModeEnablePressed = new LatchedBoolean();
    private LatchedBoolean mInPitHangModeEnablePressed = new LatchedBoolean();
    private LatchedBoolean mWOFModeEnablePressed = new LatchedBoolean();
    private boolean mInHangMode = false;
    private boolean mInWOFMode = false;
    private double mDisabledStartTime = Double.NaN;

    Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                    RobotStateEstimator.getInstance(),
                    mDrive,
                    mTurret,
                    mHood,
                    Shooter.getInstance(),
                    mSerializer,
                    mIntake,
                    mSuperstructure,
                    mLimelight,
                    mInfrastructure,
                    mCanifier,
                    mWOF
            );

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mLED.registerEnabledLoops(mEnabledLooper);
            mLED.registerEnabledLoops(mDisabledLooper);

            mTrajectoryGenerator.generateTrajectories();

            // Robot starts backwards, turret starts backwards (in robot frame)
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.fromRotation(Rotation2d.fromDegrees(180)), new Pose2d(Constants.kVehicleToTurretTranslation, Rotation2d.fromDegrees(Constants.kTurretConstants.kHomePosition)));
            mDrive.setHeading(Rotation2d.fromDegrees(180));

            mAutoModeSelector.updateModeCreator();

            mControlBoard.reset();

            mTurret.forceZero();

            mShouldNotShoot = new DelayedBoolean(Timer.getFPGATimestamp(), 0.5);
            mShouldNotAim = new DelayedBoolean(Timer.getFPGATimestamp(), 0.5);

            mSubsystemManager.stop();

            LimelightManager.getInstance().setTurretLimelight(mLimelight);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();

            mDisabledStartTime = Timer.getFPGATimestamp();
            LimelightManager.getInstance().writePeriodicOutputs();

            mLED.setWantedAction(LED.WantedAction.DISPLAY_ZEROED);

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            mDisabledLooper.stop();
            mInfrastructure.setIsTeleop(false);
            // Robot starts backwards, turret starts backwards (in robot frame)
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.fromRotation(Rotation2d.fromDegrees(180)), new Pose2d(Constants.kVehicleToTurretTranslation, Rotation2d.fromDegrees(Constants.kTurretConstants.kHomePosition)));
            mDrive.setHeading(Rotation2d.fromDegrees(180));
            mEnabledLooper.start();
            mAutoModeExecutor.start();

            mInHangMode = false;
            mLED.setWantedAction(LED.WantedAction.DISPLAY_SUPERSTRUCTURE);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {

            mInfrastructure.setIsTeleop(true);

            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mSubsystemManager.stop();

            mEnabledLooper.start();

            mInHangMode = false;
            mClimbingStateMachine.reset();
            mHangModeEnablePressed.update(true);

            mLED.setWantedAction(LED.WantedAction.DISPLAY_SUPERSTRUCTURE);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            if (mDrive.checkSystem()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            RobotState.getInstance().outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }


    @Override
    public void disabledPeriodic() {
        try {
            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            mHood.resetIfAtHome();
            if (!mHood.hasBeenZeroed()) {
                mLED.setHoodFault();
            } else {
                mLED.clearHoodFault();
            }

            mTurret.resetIfAtHome();
            if (!mTurret.hasBeenZeroed()) {
                mLED.setTurretFault();
            } else {
                mLED.clearTurretFault();
            }

            mCanifier.writePeriodicOutputs();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }

            if ((Timer.getFPGATimestamp() - mDisabledStartTime) > 5.0 &&
                    (Timer.getFPGATimestamp() - mDisabledStartTime) < 5.5) {
                System.out.println("Setting coast!");
                mClimbingStateMachine.reset();
                mDrive.setBrakeMode(false);
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopPeriodic() {
        try {

            double timestamp = Timer.getFPGATimestamp();

            boolean wants_shoot = !mShouldNotShoot.update(timestamp, !mControlBoard.getShoot());
            boolean wants_aim = !mShouldNotAim.update(timestamp, !(mControlBoard.getAimFine() || mControlBoard.getAimCoarse()));

            if (mHood.hasBeenZeroed()) {
                mLED.clearHoodFault();
            }

            if (mTurret.hasBeenZeroed()) {
                mLED.clearTurretFault();
            }

            if (mControlBoard.getZeroGyro()) {
                mDrive.setHeading(Rotation2d.fromDegrees(180));
            }

            mDrive.setHighGear(!mControlBoard.getWantsLowGear());
            mDrive.setOpenLoop(OpenLoopCheesyDriveHelper.getInstance().cheesyDrive(mControlBoard.getThrottle(),
                    mControlBoard.getTurn(), mControlBoard.getQuickTurn()));

            boolean hangModePressed =
                    mHangModeEnablePressed.update(mControlBoard.getToggleHangMode());
            boolean inPitHangModePressed = mInPitHangModeEnablePressed.update(mControlBoard.getToggleInPitHangMode());

            if ((inPitHangModePressed || hangModePressed) && !mInHangMode) {
                System.out.println("Entering hang mode!!!!");
                // Set in pit mode or regular mode
                mClimbingStateMachine.setInPitMode(inPitHangModePressed);
                mInHangMode = true;
                mLED.setWantedAction(LED.WantedAction.DISPLAY_CLIMB);
            } else if ((inPitHangModePressed || hangModePressed) && mInHangMode) {
                System.out.println("Exiting hang mode!");
                mInHangMode = false;
                mClimbingStateMachine.reset();
                mLED.setWantedAction(LED.WantedAction.DISPLAY_SUPERSTRUCTURE);
            }

            boolean WOFModePressed = mWOFModeEnablePressed.update(mControlBoard.getToggleWOFMode());
            if (WOFModePressed && !mInWOFMode) {
                System.out.println("Entering WOF mode!!!!");
                mInWOFMode = true;
                mLED.setWOFLEDState(TimedLEDState.BlinkingLEDState.kBlinkingWOF);
                mLED.setWantedAction(LED.WantedAction.DISPLAY_WOF);
            } else if (WOFModePressed && mInWOFMode) {
                System.out.println("Exiting WOF mode!");
                mInWOFMode = false;
                mLED.setWantedAction(LED.WantedAction.DISPLAY_SUPERSTRUCTURE);
            }


            if (mInWOFMode) {
                mWOF.setDeploy(true);
                mWOF.setOpenLoop(mControlBoard.getStir() * 0.5);
            } else {
                mWOF.setDeploy(false);
            }


            if (mInHangMode) {
                mSuperstructure.setOverrideLimelightLEDs(true);
                LimelightManager.getInstance().getTurretLimelight().setLed(Limelight.LedMode.BLINK);
                mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
                mClimbingStateMachine.handle(Timer.getFPGATimestamp(), mControlBoard.getClimbJog(), false, mControlBoard.getRetractIntake(),
                        mControlBoard.getHumanPlayerIntake(), mControlBoard.getDeployIntake());
                mSuperstructure.setTurretHintRobotRelative(Timer.getFPGATimestamp(), -90);


            } else {
                mSuperstructure.setOverrideLimelightLEDs(false);
                if (Math.abs(mControlBoard.getStir()) > Constants.kSerializerStirDeadband && !mInWOFMode) {
                    mSerializer.setStirOverriding(true);
                    mSerializer.setOpenLoop(Util.handleDeadband(mControlBoard.getStir(),
                            Constants.kSerializerStirDeadband) * Constants.kSerializerStirScalar);
                } else {
                    mSerializer.setStirOverriding(false);
                }

                mSerializer.setSerializerCanceled(mControlBoard.getCancelAutoSerialize());

                if (mControlBoard.getExhaust()) {
                    mIntake.setWantedState(Intake.WantedState.EXHAUST);
                } else if (mControlBoard.getIntake()) {
                    mIntake.setWantedState(Intake.WantedState.INTAKE);
                } else {
                    mIntake.setWantedState(Intake.WantedState.IDLE);
                }

                if (mControlBoard.getHoodJog() > 0.5) {
                    mSuperstructure.setBallQuality(BallQuality.OLD_BALL);
                } else if (mControlBoard.getHoodJog() < -0.5) {
                    mSuperstructure.setBallQuality(BallQuality.NEW_BALL);
                } else {
                    mSuperstructure.setBallQuality(BallQuality.MEDIUM_BALL);
                }

                if (wants_shoot && wants_aim) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
                } else if (wants_aim) {
                    mSuperstructure.setShootingParams(mControlBoard.getAimFine() ? Constants.kFineShootingParams : Constants.kCoarseShootingParams);
                    mSuperstructure.setWantedState(Superstructure.WantedState.AIM);
                } else if (mControlBoard.getMoveToZero()) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.MOVE_TO_ZERO);
                } else {
                    mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
                }

                if (mControlBoard.getTurretHint() != CardinalDirection.NONE) {
                    mSuperstructure.resetTurretJog();
                    mSuperstructure.setTurretHint(mControlBoard.getTurretHint().getRotation().getDegrees());
                } else if (mControlBoard.getTurretJog() != 0.0) {
                    mSuperstructure.resetTurretHint();
                    mSuperstructure.setTurretJog(mControlBoard.getTurretJog() * Constants.kJogTurretScalar);
                } else {
                    mSuperstructure.resetTurretHint();
                    mSuperstructure.resetTurretJog();
                }

                // Intake
                if (mControlBoard.getIntake()) {
                    mIntake.deploy();
                } else {
                    mIntake.stow();
                }
            }

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}
}
