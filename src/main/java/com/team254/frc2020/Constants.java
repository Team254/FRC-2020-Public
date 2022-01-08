package com.team254.frc2020;

import com.team254.frc2020.limelight.CameraResolution;
import com.team254.frc2020.limelight.PipelineConfiguration;
import com.team254.frc2020.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.ShootingParameters;
import edu.wpi.first.wpilibj.util.Units;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 * <p>
 * Port assignments should match up with the spreadsheet here:
 * https://docs.google.com/spreadsheets/d/1U1r9AyXk8nuGuACa36iQRRekL6HqLXF7dZ437_qiD98/edit?usp=sharing
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    // CAN
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    // Control board
    public static final boolean kUseDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.2;
    public static final double kJogTurretScalar = 4.0;

    // Drivebase
    // ids
    public static final int kLeftDriveMaster1Id = 1;
    public static final int kLeftDriveMaster2Id = 2;
    public static final int kLeftDriveMaster3Id = 3;
    public static final int kRightDriveMaster1Id = 4;
    public static final int kRightDriveMaster2Id = 5;
    public static final int kRightDriveMaster3Id = 6;

    public static final int kLeftDriveEncoderA = 0;
    public static final int kLeftDriveEncoderB = 1;
    public static final int kRightDriveEncoderA = 2;
    public static final int kRightDriveEncoderB = 3;

    // Drive ratio.
    public static final double kDriveEncoderPPR = 1000.0;
    public static final double kDriveLowGearReduction = 40.0 / 11.0 * 44.0 / 20.0;
    public static final double kDriveHighGearReduction = 40.0 / 11.0 * 50.0 / 14.0;
    public static final double kDriveRotationsPerTickHighGear = 1.0 / 2048.0 * 1.0 / kDriveLowGearReduction; // ticks * kDriveRotationsPerTicksHighGear = wheel rotations
    public static final double kDriveRotationsPerTickLowGear = 1.0 / 2048.0 * 1.0 / kDriveHighGearReduction; // ticks * kDriveRotationsPerTicksLowGear = wheel rotations
    public static final double kGearRatioScalar = (1.0 / (40.0 / 10.0 * 50.0 / 14.0)) / (1.0 / kDriveHighGearReduction);
    // Wheel parameters.
    public static final double kDriveWheelTrackWidthInches = 30.0; //tuned 3/2
    public static final double kDriveWheelDiameterInches = 5.9067052758; //tuned 3/2
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelRadiusMeters = Units.inchesToMeters(kDriveWheelDiameterInches);
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 1.0;

    // pidf gains (TODO tune)
    public static final double kDriveHighGearKp = 0.15;
    public static final double kDriveHighGearKi = 0.0;
    public static final double kDriveHighGearKd = 0.0;
    public static final double kDriveHighGearKf = 0.0;

    public static final double kDriveLowGearKp = 0.0;
    public static final double kDriveLowGearKi = 0.0;
    public static final double kDriveLowGearKd = 0.0;
    public static final double kDriveLowGearKf = 0.0;

    public static final double kDrivePositionKp = 0.006;
    public static final double kDrivePositionKi = 0.0;
    public static final double kDrivePositionKd = 0.0;
    public static final double kDrivePositionKf = 0.0;

    // deadband
    public static final double kDriveThrottleDeadband = 0.04;
    public static final double kDriveWheelDeadband = 0.035;


    // robot dynamics TODO tune
    public static final double kDriveVIntercept = 0.352; // V TODO
    public static final double kDriveLinearKv = 0.0438 / 2.0 * Constants.kDriveWheelDiameterInches; // V / rad/s
    public static final double kFalcon500StallTorque = 4.69; // N*m
    public static final double kAssumedTorqueEfficiency = 0.95;
    public static final double kRobotLinearInertia = 62.051; // kg TODO
    public static final double kDriveAnalyticalLinearKa = 12.0 /* V */ / ((kDriveHighGearReduction * kFalcon500StallTorque * kAssumedTorqueEfficiency * 6) / (kRobotLinearInertia * kDriveWheelRadiusMeters * kDriveWheelRadiusMeters));
    public static final double kDriveLinearKa = 0.00597 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V / rad/s^2
    public static final double kDriveAngularKa = 0.00517 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V per rad/s^2
    public static final double kRobotAngularInertia = kDriveAngularKa / kDriveLinearKa *
            kDriveWheelTrackRadiusWidthMeters * kDriveWheelTrackRadiusWidthMeters * kRobotLinearInertia;  // kg m^2
    public static final double kRobotAngularDrag = 15.0; // N*m / (rad/sec)

    // path following TODO tune?
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    // Turret.
    public static final ServoMotorSubsystemConstants kTurretConstants = new ServoMotorSubsystemConstants();
    static {
        kTurretConstants.kName = "Turret";

        kTurretConstants.kMasterConstants.id = 10;
        kTurretConstants.kMasterConstants.invert_motor = false;

        // Unit == Degrees
        kTurretConstants.kHomePosition = 180.0;
        kTurretConstants.kTicksPerUnitDistance = 1.0 / (1.0 / 2048.0 * 8.0 / 24.0 * 14.0 / 240.0 * 360.0);

        kTurretConstants.kKf = 1023.0 * 0.25 / 4650.0; // Tuned 3/4
        kTurretConstants.kPositionKp = 0.21;
        kTurretConstants.kPositionDeadband = (int) (0.1 * kTurretConstants.kTicksPerUnitDistance); // Ticks

        kTurretConstants.kMinUnitsLimit = 45.0;
        kTurretConstants.kMaxUnitsLimit = 315.0;

        // TODO current limits, should recover position on reset?

        kTurretConstants.kRecoverPositionOnReset = true;
    }

    public static final Translation2d kVehicleToTurretTranslation = new Translation2d(-6.9, 0);

    // Hood
    public static final ServoMotorSubsystemConstants kHoodConstants = new ServoMotorSubsystemConstants();
    static {
        kHoodConstants.kName = "Hood";

        kHoodConstants.kMasterConstants.id = 11;
        kHoodConstants.kMasterConstants.invert_motor = true;

        // Unit == Degrees
        kHoodConstants.kHomePosition = 45.0;
        kHoodConstants.kTicksPerUnitDistance = 1.0 / (1.0 / 2048.0 * 8.0 / 40.0 * 16.0 / 38.0 * 14.0 / 366.0 * 360.0);

        kHoodConstants.kPositionKp = 0.254; // TODO (tune better)
        kHoodConstants.kPositionDeadband = (int) (0.1 * kHoodConstants.kTicksPerUnitDistance); // Ticks

        kHoodConstants.kMinUnitsLimit = 45.0;
        kHoodConstants.kMaxUnitsLimit = 70.0;

        // TODO current limits
        kHoodConstants.kRecoverPositionOnReset = true;
    }

    // Shooter
    public static final int kShooterLeftMasterId = 12;
    public static final int kShooterRightMasterId = 13;
    public static final double kShooterKp = 0.0075;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 0.0;
    public static final double kShooterKf = 0.05033127788;
    public static final double kShooterTicksPerRevolution = 2048.0; // based on gear reduction between encoder and output shaft, and encoder ppr
    public static final double kShooterAllowableErrorRPM = 250.0; // TODO

    // Serializer
    public static final int kSerializerSpinCycleMasterId = 7;
    public static final int kSerializerLeftRollerMasterId = 8;
    public static final int kSerializerRightRollerMasterId = 9;

    public static final double kSerializerStirDeadband = 0.3;
    public static final double kSerializerStirScalar = 0.5;

    public static final double kFeederRollersKp = 0.075;
    public static final double kFeederRollersKi = 0.0;
    public static final double kFeederRollersKd = 0.2;
    public static final double kFeederRollersKf = 0.04795003;
    public static final double kFeederRollersTicksPerRevolutions = 2048.0 * 1.0 / 36.0 * 16.0;
    public static final int kFeederAllowableError = 50;

    // Intake
    public static final int kIntakeMasterId = 14;

    // WOF
    public static final int kWOFSpinnerId = 15;

    // Canifier
    public static final int kCanifierId = 16;

    // Pigeon IMU
    public static final int kPigeonIMUId = 0;

    // Pneumatics
    public static final int kPCMId = 0;
    public static final int kShifterSolenoidId = 0;
    public static final int kPTOSolenoidId = 1;
    public static final int kIntakeSolenoidId = 2;
    public static final int kSkateParkSolenoidId = 5;
    public static final int kChockSolenoidId = 7;
    public static final int kBrakeSolenoidId = 6;
    public static final int kDeploySolenoidId = 3;
    public static final int kWOFSolenoidId = 4;

    // Vision
    public static final boolean kUseTopCorners = false;

    public static final double kTurretLimelightLensOffGroundHeight = 26.48;
    public static final Rotation2d kTurretLimelightHorizontalPlaneToLens = Rotation2d.fromDegrees(27.00);

    public static final boolean kShouldUndistort = true;
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kCameraFrameRate = 90.0; // fps

    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final double kTopVisionTargetHeight = 98.25;
    public static final double kBottomVisionTargetHeight = 81.25;

    public static final Pose2d kVisionTargetToGoalOffset = new Pose2d(-29.25, 0, Rotation2d.identity());

    public static final PipelineConfiguration kLowRes1xZoom = new PipelineConfiguration(CameraResolution.F_320x240, 1.0);
    public static final PipelineConfiguration kLowRes2xZoom = new PipelineConfiguration(CameraResolution.F_320x240, 2.0);

    // Shot tuning
    public static final double kDefaultShooterRPM = 4500;
    public static final boolean kIsHoodTuning = false;
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLaserOldHoodMap = new InterpolatingTreeMap<>();
    static {
        // tuned 3/8
        kLaserOldHoodMap.put(new InterpolatingDouble(105.171167), new InterpolatingDouble(45.729859));
        kLaserOldHoodMap.put(new InterpolatingDouble(126.321321), new InterpolatingDouble(48.132334));
        kLaserOldHoodMap.put(new InterpolatingDouble(140.935187), new InterpolatingDouble(50.033137));
        kLaserOldHoodMap.put(new InterpolatingDouble(167.806665), new InterpolatingDouble(52.024536));
        kLaserOldHoodMap.put(new InterpolatingDouble(183.306690), new InterpolatingDouble(54.025561));
        kLaserOldHoodMap.put(new InterpolatingDouble(214.597001), new InterpolatingDouble(56.017068));
        kLaserOldHoodMap.put(new InterpolatingDouble(226.597001), new InterpolatingDouble(56.78082148));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLaserNewHoodMap = new InterpolatingTreeMap<>();
    static {
        // tuned 3/8
        kLaserNewHoodMap.put(new InterpolatingDouble(91.399880), new InterpolatingDouble(45.759868));
        kLaserNewHoodMap.put(new InterpolatingDouble(96.723329), new InterpolatingDouble(46.792655));
        kLaserNewHoodMap.put(new InterpolatingDouble(110.116684), new InterpolatingDouble(50.354185));
        kLaserNewHoodMap.put(new InterpolatingDouble(120.648394), new InterpolatingDouble(52.094181));
        kLaserNewHoodMap.put(new InterpolatingDouble(128.497332), new InterpolatingDouble(54.010839));
        kLaserNewHoodMap.put(new InterpolatingDouble(142.010190), new InterpolatingDouble(56.069618));
        kLaserNewHoodMap.put(new InterpolatingDouble(168.366039), new InterpolatingDouble(57.976650));
        kLaserNewHoodMap.put(new InterpolatingDouble(183.160319), new InterpolatingDouble(60.119230));
        kLaserNewHoodMap.put(new InterpolatingDouble(244.351501), new InterpolatingDouble(61.206940));
        kLaserNewHoodMap.put(new InterpolatingDouble(263.950477), new InterpolatingDouble(62.035888));
        kLaserNewHoodMap.put(new InterpolatingDouble(275.950477), new InterpolatingDouble(62.54343370034679));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLaserMediumHoodMap = new InterpolatingTreeMap<>();
    static {
        // tuned 3/8
        kLaserMediumHoodMap.put(new InterpolatingDouble(302.86658), new InterpolatingDouble(61.56313840676865));
        kLaserMediumHoodMap.put(new InterpolatingDouble(290.866580), new InterpolatingDouble(61.446452));
        kLaserMediumHoodMap.put(new InterpolatingDouble(248.999184), new InterpolatingDouble(61.039339));
        kLaserMediumHoodMap.put(new InterpolatingDouble(179.368013), new InterpolatingDouble(58.555894));
        kLaserMediumHoodMap.put(new InterpolatingDouble(151.1), new InterpolatingDouble(53.9));
        kLaserMediumHoodMap.put(new InterpolatingDouble(124.535161), new InterpolatingDouble(51.917520));
        kLaserMediumHoodMap.put(new InterpolatingDouble(101.349366), new InterpolatingDouble(47.373032));
        kLaserMediumHoodMap.put(new InterpolatingDouble(101.349366), new InterpolatingDouble(47.373032));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLaserRPMMap = new InterpolatingTreeMap<>();
    static {
        kLaserRPMMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(4500.0));
    }

    public static final ShootingParameters kCoarseShootingParams = new ShootingParameters(
            kLaserOldHoodMap, // old hood map
            kLaserMediumHoodMap, // medium hood map
            kLaserNewHoodMap, // new hood map
            kLaserRPMMap, // rpm map
            Pose2d.identity(), // vision target to goal offset
            0.75, // spin cycle setpoint (percent output)
            100, // shooter allowable error (rpm)
            1.0, // turret allowable error (°)
            0.5 // hood allowable error (°)
    );

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLobHoodMap = new InterpolatingTreeMap<>();
    static {
        kLobHoodMap.put(new InterpolatingDouble(172.149916), new InterpolatingDouble(48.750000));
        kLobHoodMap.put(new InterpolatingDouble(208.664221), new InterpolatingDouble(50.185451));
        kLobHoodMap.put(new InterpolatingDouble(248.562343), new InterpolatingDouble(53.421403));
        kLobHoodMap.put(new InterpolatingDouble(274.035243), new InterpolatingDouble(55.572476));
        kLobHoodMap.put(new InterpolatingDouble(316.418550), new InterpolatingDouble(57.561044));
        kLobHoodMap.put(new InterpolatingDouble(328.41855), new InterpolatingDouble(58.124067928264964));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLobRPMMap = new InterpolatingTreeMap<>();
    static {
        kLobRPMMap.put(new InterpolatingDouble(172.149916), new InterpolatingDouble(2500.0));
        kLobRPMMap.put(new InterpolatingDouble(208.664221), new InterpolatingDouble(2550.0));
        kLobRPMMap.put(new InterpolatingDouble(248.562343), new InterpolatingDouble(2750.0));
        kLobRPMMap.put(new InterpolatingDouble(274.035243), new InterpolatingDouble(2850.0));
        kLobRPMMap.put(new InterpolatingDouble(316.418550), new InterpolatingDouble(3150.0));
    }

    public static final ShootingParameters kFineShootingParams = new ShootingParameters(
            kLobHoodMap, // old hood map
            kLobHoodMap, // medium hood map
            kLobHoodMap, // new hood map
            kLobRPMMap, // rpm map
            Constants.kVisionTargetToGoalOffset, // vision target to goal offset
            0.5, // spin cycle setpoint (percent output)
            100, // shooter allowable error (rpm)
            1.0, // turret allowable error (°)
            0.5 // hood allowable error (°)
    );
}