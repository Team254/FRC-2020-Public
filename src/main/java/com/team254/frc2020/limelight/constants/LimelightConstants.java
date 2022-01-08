package com.team254.frc2020.limelight.constants;

import com.team254.frc2020.limelight.CameraResolution;
import com.team254.frc2020.limelight.undistort.OpenCVCalculatedUndistortMap;
import com.team254.frc2020.limelight.undistort.UndistortConstants;
import com.team254.frc2020.limelight.undistort.UndistortMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class LimelightConstants {
    private int id;
    private String name;
    private String tableName;
    private double height;
    private Pose2d turretToLens;
    private Rotation2d horizontalPlaneToLens;
    private UndistortMap undistortMap;
    private UndistortConstants undistortConstants;
    private Type type;
    private double horizontalFOV;
    private double verticalFOV;

    public enum Type {
        Shooter
    }

    public LimelightConstants(int id, Type type, String name, String tableName, double height, Pose2d turretToLens, Rotation2d horizontalPlaneToLens, UndistortMap undistortMap, UndistortConstants undistortConstants, double horizontalFOV, double verticalFOV) {
        this.id = id;
        this.type = type;
        this.name = name;
        this.tableName = tableName;
        this.height = height;
        this.turretToLens = turretToLens;
        this.horizontalPlaneToLens = horizontalPlaneToLens;
        this.undistortMap = undistortMap;
        this.undistortConstants = undistortConstants;
        if (this.undistortMap == null) {
            this.undistortMap = new OpenCVCalculatedUndistortMap(undistortConstants, CameraResolution.F_320x240, false);
        }
        this.horizontalFOV = horizontalFOV;
        this.verticalFOV = verticalFOV;
    }

    public int getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public String getTableName() {
        return tableName;
    }

    public double getHeight() {
        return height;
    }

    public Pose2d getTurretToLens() {
        return turretToLens;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return horizontalPlaneToLens;
    }

    public UndistortMap getUndistortMap() {
        return undistortMap;
    }

    public UndistortConstants getUndistortConstants() {
        return undistortConstants;
    }

    public Type getType() {
        return type;
    }

    public double getHorizontalFOV() {
        return horizontalFOV;
    }

    public double getVerticalFOV() {
        return verticalFOV;
    }
}