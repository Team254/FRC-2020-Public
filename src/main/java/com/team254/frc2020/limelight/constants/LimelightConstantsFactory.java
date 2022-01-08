package com.team254.frc2020.limelight.constants;

import com.team254.frc2020.Constants;
import com.team254.frc2020.RobotType;
import com.team254.frc2020.limelight.undistort.UndistortConstants;
import com.team254.frc2020.limelight.undistort.precomputedmaps.UndistortMap_Limelight_0_320x240;
import com.team254.frc2020.limelight.undistort.precomputedmaps.UndistortMap_Limelight_1_320x240;
import com.team254.frc2020.limelight.undistort.precomputedmaps.UndistortMap_Limelight_2_320x240;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class LimelightConstantsFactory {
    public static LimelightConstants getConstantsForId(int id) {
        switch (id) {
            default: // Intentional fall through
            case 0: // Limelight used for vision tests
                return new LimelightConstants(
                        0, // label id
                        null,
                        "Test Limelight", // name
                        "limelight", //table name
                        14.125, // height
                        Pose2d.identity(), // turret to lens
                        Rotation2d.fromDegrees(1.6), // horizontalPlaneToLens
                        new UndistortMap_Limelight_0_320x240(), // undistort map
                        new UndistortConstants( // undistort constants
                                new double[]{2.35265164e-01, -6.86035030e-01, 3.10037972e-04, 8.34493852e-05, 6.41764110e-01},
                                new double[][]{
                                        {778.08226793 / 960, 0, 452.8538724 / 960},
                                        {0, 777.04925262 / 720, 351.05573266 / 720},
                                        {0, 0, 1.0}
                                }
                        ),
                        63.709400992230975,
                        49.71097153877655
                );
            case 1:
                return new LimelightConstants(
                        1, // label id
                        LimelightConstants.Type.Shooter,
                        "Turret Limelight #1", // name
                        "limelight", // table name
                        Constants.kTurretLimelightLensOffGroundHeight, // height
                        new Pose2d(-5.7, 0, Rotation2d.fromDegrees(1.0)), // turret to lens
                        Constants.kTurretLimelightHorizontalPlaneToLens, // horizontalPlaneToLens,
                        new UndistortMap_Limelight_1_320x240(), // undistort map
                        new UndistortConstants( // undistort constants
                                new double[]{2.03204609e-01, -6.25404962e-01, -3.39277869e-03, -3.51126715e-04, 5.81122457e-01}, // camera distortion
                                new double[][]{ // camera matrix
                                        {0.78474188, 0.0, 0.51036895},
                                        {0.0, 1.04536274, 0.45914132},
                                        {0.0, 0.0, 1.0}
                                }),
                        65.00022891576718,
                        51.06843506401144
                );

            case 2:
                return new LimelightConstants(
                        2, // label id
                        LimelightConstants.Type.Shooter,
                        "Turret Limelight #2", // name
                        "limelight", // table name
                        Constants.kTurretLimelightLensOffGroundHeight, // height
                        new Pose2d(-5.7, 0, Rotation2d.fromDegrees(1.5)), // turret to lens
                        Constants.kTurretLimelightHorizontalPlaneToLens, // horizontalPlaneToLens,
                        new UndistortMap_Limelight_2_320x240(),
                        new UndistortConstants( // undistort constants
                                new double[]{2.08955661e-01, -6.41863833e-01, 6.17627447e-04, -6.37834053e-05, 6.35575213e-01}, // camera distortion
                                new double[][]{ // camera matrix
                                        {0.79943057, 0.0, 0.51249286},
                                        {0.0, 1.06369973, 0.51129839},
                                        {0.0, 0.0, 1.0}
                                }),
                        64.03840065743408,
                        50.34836606499798
                );

            case 3:
                return new LimelightConstants(
                        3, // label id
                        LimelightConstants.Type.Shooter,
                        "Intake Limelight #3", // name
                        "limelight", // table name
                        Constants.kTurretLimelightLensOffGroundHeight, // height
                        new Pose2d(-5.7, 0, Rotation2d.fromDegrees(1.5)), // turret to lens
                        Constants.kTurretLimelightHorizontalPlaneToLens, // horizontalPlaneToLens
                        null,   // TODO generate map
                        new UndistortConstants( // undistort constants
                                new double[]{0.20120696, -0.54084708, 0.0014172, -0.00512332, 0.46871865}, // camera distortion
                                new double[][]{ // camera matrix
                                        {0.8027808, 0.0, 0.47932945},
                                        {0.0, 1.07363039, 0.51292353},
                                        {0.0, 0.0, 1.0}
                                }),
                        63.807633432913114,
                        49.93847526720772
                );

            case 4:
                return new LimelightConstants(
                        4, // label id
                        LimelightConstants.Type.Shooter,
                        "Intake Limelight #4", // name
                        "limelight", // table name
                        Constants.kTurretLimelightLensOffGroundHeight, // height
                        new Pose2d(-5.7, 0, Rotation2d.fromDegrees(1.5)), // turret to lens
                        Constants.kTurretLimelightHorizontalPlaneToLens, // horizontalPlaneToLens
                        null,   // TODO generate map
                        new UndistortConstants( // undistort constants
                                new double[]{2.08955661e-01, -6.41863833e-01, 6.17627447e-04, -6.37834053e-05, 6.35575213e-01}, // camera distortion
                                new double[][]{ // camera matrix
                                        {0.79943057, 0.0, 0.51249286},
                                        {0.0, 1.06369973, 0.51129839},
                                        {0.0, 0.0, 1.0}
                                }),
                        64.03840065743408,
                        50.34836606499798
                );

            case 5:
                return new LimelightConstants(
                        5, // label id
                        LimelightConstants.Type.Shooter,
                        "Limelight #5", // name
                        "limelight", // table name
                        Constants.kTurretLimelightLensOffGroundHeight, // height
                        new Pose2d(-5.7, 0, Rotation2d.fromDegrees(1.5)), // turret to lens
                        Constants.kTurretLimelightHorizontalPlaneToLens, // horizontalPlaneToLens
                        null,   // TODO generate map
                        new UndistortConstants( // undistort constants
                                new double[]{0.20014465, -0.47922169, -0.00415085, -0.00084271, 0.35440957}, // camera distortion
                                new double[][]{ // camera matrix
                                        {0.82360559, 0.0, 0.47004554},
                                        {0.0, 1.09854666, 0.45221907},
                                        {0.0, 0.0, 1.0}
                                }),
                        62.473651304991805,
                        48.877367093835794
                );

            case 6:
                return new LimelightConstants(
                        6, // label id
                        LimelightConstants.Type.Shooter,
                        "Limelight #6", // name
                        "limelight", // table name
                        Constants.kTurretLimelightLensOffGroundHeight, // height
                        new Pose2d(-5.7, 0, Rotation2d.fromDegrees(1.5)), // turret to lens
                        Constants.kTurretLimelightHorizontalPlaneToLens, // horizontalPlaneToLens
                        null,   // TODO generate map
                        new UndistortConstants( // undistort constants
                                new double[]{1.90847215e-01, -5.13780075e-01, -2.34747538e-04, -2.56021324e-03, 4.27250411e-01}, // camera distortion
                                new double[][]{ // camera matrix
                                        {0.81140397, 0.0, 0.46869083},
                                        {0.0, 1.08436796, 0.48825234},
                                        {0.0, 0.0, 1.0}
                                }),
                        63.228865885979154,
                        49.50445641798894
                );
        }
    }

    public static LimelightConstants getConstantsForThisRobot() {
        RobotType.Type type = RobotType.getRobotType();
        int id = getIdForRobotType(type);
        return getConstantsForId(id);
    }

    public static int getIdForRobotType(RobotType.Type type) {
        if (type == RobotType.Type.PRACTICE) {
            return 1;
        }
        return 2;
    }
}
