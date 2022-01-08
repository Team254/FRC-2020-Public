package com.team254.frc2020.limelight.undistort;

import com.team254.frc2020.limelight.CameraResolution;
import com.team254.lib.util.Util;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.opencv.core.CvType.CV_64FC1;
import static org.opencv.core.CvType.CV_64FC2;

public class OpenCVCalculatedUndistortMap implements UndistortMap {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private CameraResolution cameraResolution;
    private UndistortConstants undistortConstants;
    double[][][] map;

    private AtomicBoolean ready = new AtomicBoolean(false);
    private AtomicBoolean loaded = new AtomicBoolean(false);

    public OpenCVCalculatedUndistortMap(UndistortConstants undistortConstants, CameraResolution cameraResolution, boolean loadAsync) {
        this.cameraResolution = cameraResolution;
        this.undistortConstants = undistortConstants;
        this.map = new double[cameraResolution.getWidth()][cameraResolution.getHeight()][2];
        if (loadAsync) {
            new Thread(this::load).start();
        } else {
            load();
        }
    }

    private void load() {
        try {
            long startMillis = System.currentTimeMillis();
            for (int i = 0; i < cameraResolution.getWidth(); i++) {
                for (int j = 0; j < cameraResolution.getHeight(); j++) {
                    // run undistort
                    double[] point = new double[]{i * 1.0 / cameraResolution.getWidth() * 1.0, j * 1.0 / cameraResolution.getHeight() * 1.0};
                    double[] undistorted = undistortFromOpenCV(undistortConstants, point);
                    // output undistorted x and y
                    for (int k = 0; k < 2; k++) {
                        map[i][j][k] = (float) undistorted[k];
                    }
                }
            }
            loaded.set(true);
            System.out.println("!!!!!!!!!! Undistort Map Loaded in " + ((System.currentTimeMillis() - startMillis) / 1000.0) + " seconds !!!!!!!!!!");
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            ready.set(true);
        }
    }

    @Override
    public double[] getUndistortedPoint(double x, double y) {
        if (ready.get()) {
            double denormalizedX = x * cameraResolution.getWidth();
            double denormalizedY = y * cameraResolution.getHeight();
            int intX = (int) Util.limit(denormalizedX, 0, cameraResolution.getWidth());
            int intY = (int) Util.limit(denormalizedY, 0, cameraResolution.getHeight());
            return map[intX][intY]; // map holds normalized output values, keyed by ints
        } else {
            // Return an un-undistorted point until this is loaded.
            double[] ret = {x, y};
            return ret;
        }
    }

    public double[] getUndistortedPointRawPixelSpace(double x, double y) {
        if (ready.get()) {
            int intX = (int) Util.limit(x, 0, cameraResolution.getWidth());
            int intY = (int) Util.limit(y, 0, cameraResolution.getHeight());
            return map[intX][intY]; // map holds normalized output values, keyed by ints
        } else {
            // Return an un-undistorted point until this is loaded.
            double[] ret = {x, y};
            return ret;
        }
    }

    /**
     * Undoes radial and tangential distortion using opencv
     */
    private static double[] undistortFromOpenCV(UndistortConstants undistortConstants, double[] point) {
        Mat coord = new Mat(1, 1, CV_64FC2);
        coord.put(0, 0, point);

        Mat camMtx = new Mat(3, 3, CV_64FC1);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                camMtx.put(i, j, undistortConstants.getCameraMatrix()[i][j]);
            }
        }

        Mat distortion = new Mat(1, 5, CV_64FC1);
        for (int i = 0; i < 5; i++) {
            distortion.put(0, i, undistortConstants.getCameraDistortion()[i]);
        }

        Mat dst = new Mat(1, 1, CV_64FC2);

        Imgproc.undistortPoints(coord, dst, camMtx, distortion, new Mat(), camMtx);

        return dst.get(0, 0);
    }

    @Override
    public boolean getReady() {
        return ready.get();
    }

    public boolean getLoaded() {
        return loaded.get();
    }
}
