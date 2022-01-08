package com.team254.frc2020.limelight;

public enum CameraResolution {
    F_320x240(320, 240),
    F_960x720(960, 720);

    final int width;
    final int height;

    CameraResolution(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }
}