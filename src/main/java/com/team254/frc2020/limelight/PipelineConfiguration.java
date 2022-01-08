package com.team254.frc2020.limelight;

public class PipelineConfiguration {
    public final CameraResolution fullFrameCameraResolution;
    public final double zoomFactor;
    public final double crosshairX;
    public final double crosshairY;

    private final double offsetX;
    private final double offsetY;

    public PipelineConfiguration(CameraResolution fullCameraResolution, double zoomFactor) {
        this(fullCameraResolution, zoomFactor, 0.5, 0.5);
    }

    public PipelineConfiguration(CameraResolution fullFrameCameraResolution, double zoomFactor, double crosshairX, double crosshairY) {
        this.fullFrameCameraResolution = fullFrameCameraResolution;
        this.zoomFactor = Math.round(zoomFactor);
        this.crosshairX = crosshairX;
        this.crosshairY = crosshairY;

        offsetX = crosshairX - 1.0 / (2.0 * zoomFactor);
        offsetY = crosshairY - 1.0 / (2.0 * zoomFactor);
    }

    public double[] normalize(double[] point) {
        double[] transformed = new double[2];

        transformed[0] = point[0] / fullFrameCameraResolution.width / zoomFactor + offsetX;
        transformed[1] = point[1] / fullFrameCameraResolution.height / zoomFactor + offsetY;

        return transformed;
    }
}