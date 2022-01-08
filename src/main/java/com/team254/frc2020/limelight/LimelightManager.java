package com.team254.frc2020.limelight;

import com.team254.frc2020.subsystems.Limelight;

public class LimelightManager {

    private Limelight turretLimelight;

    private static LimelightManager mInstance;

    public static LimelightManager getInstance() {
        if (mInstance == null) {
            mInstance = new LimelightManager();
        }
        return mInstance;
    }


    private LimelightManager() {}

    public void setTurretLimelight(Limelight limelight) {
        turretLimelight = limelight;
    }

    public Limelight getTurretLimelight() {
        return turretLimelight;
    }

    public void writePeriodicOutputs() {
        turretLimelight.writePeriodicOutputs();
    }
}
