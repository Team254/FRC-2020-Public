package com.team254.frc2020.limelight;

import com.team254.frc2020.limelight.constants.LimelightConstants;
import com.team254.frc2020.limelight.constants.LimelightConstantsFactory;
import com.team254.frc2020.limelight.undistort.OpenCVCalculatedUndistortMap;
import org.junit.Test;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class UndistortMapFileWriterTest {
    private static final boolean DO_WRITE_UNDISTORT_MAP = false;
    private static final CameraResolution K_UNDISTORT_MAP_WRITER_RES = CameraResolution.F_320x240;

    @Test
    public void testMakeUndistortMap() throws IOException {
        if (!DO_WRITE_UNDISTORT_MAP) {
            return;
        }

        // SET THESE 2 things
        int limelightId = 1;
        CameraResolution resolution = K_UNDISTORT_MAP_WRITER_RES;


        // Don't edit below this
        LimelightConstants constants = LimelightConstantsFactory.getConstantsForId(limelightId);
        String varName = "map";
        StringBuilder sb = new StringBuilder();

        String className = String.format("UndistortMap_Limelight_%d_%dx%d", limelightId, resolution.getWidth(), resolution.getHeight());
        // Header
        sb.append("package com.team254.frc2020.limelight.undistort.precomputedmaps;\n" +
                "\n" +
                "import com.team254.frc2020.limelight.undistort.UndistortMap;\n\n" +
                String.format("public class %s implements UndistortMap {\n", className));


        long start = System.currentTimeMillis();
        OpenCVCalculatedUndistortMap map = new OpenCVCalculatedUndistortMap(constants.getUndistortConstants(), resolution, false);
        long end = System.currentTimeMillis();
        System.out.println("Delta " + (end - start));

        // Write overall variable
        String variableLine = String.format("  double[][][] %s = new double[%d][%d][2];\n", varName, resolution.getWidth(), resolution.getHeight());
        sb.append(variableLine);

        // Write Y column loader methods
        for (int x = 0; x < resolution.getWidth(); x++) {
            sb.append(String.format("  public double[][] loadCol%03d() { ", x));
            ArrayList<String> lineParts = new ArrayList<>();
            for (int y = 0; y < resolution.getHeight(); y++) {
                double[] pt = map.getUndistortedPointRawPixelSpace(x, y);
                String strPt = String.format("{%.3f,%.3f}", pt[0], pt[1]);
                lineParts.add(strPt);
            }
            String numbers = String.join(",", lineParts);
            sb.append(String.format("return new double[][]{%s};", numbers));
            sb.append(" }\n");
        }

        // Write static loader
        sb.append("\n");
        sb.append(String.format("  public %s() {\n", className));
        for (int x = 0; x < resolution.getWidth(); x++) {
            String loadLine = String.format("    map[%d] = loadCol%03d();\n", x, x);
            sb.append(loadLine);
        }
        // End static loader
        sb.append("  }\n");

        // Write getters
        sb.append("\n");
        sb.append("  @Override\n");
        sb.append("  public double[] getUndistortedPoint(double x, double y) {\n");
        sb.append("    int denormalizedX = (int) (x * " + resolution.getWidth() + ");\n");
        sb.append("    int denormalizedY = (int) (y * " + resolution.getHeight() + ");\n");
        sb.append("    return map[denormalizedX][denormalizedY];\n");
        sb.append("  }\n\n");

        sb.append("  @Override\n");
        sb.append("  public boolean getReady() {\n");
        sb.append("    return true;\n");
        sb.append("  }\n\n");

        sb.append("}\n");

        BufferedWriter writer = new BufferedWriter(new FileWriter(String.format("/tmp/%s.java", className)));
        writer.write(sb.toString());
        writer.close();
    }
}
