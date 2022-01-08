package com.team254.lib.trajectory;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.util.Arrays;
import java.util.List;

@RunWith(JUnit4.class)
public class DistanceViewTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // Specify desired waypoints.
        List<Translation2d> waypoints = Arrays.asList(
                new Translation2d(0.0, 0.0),
                new Translation2d(24.0, 0.0),
                new Translation2d(36.0, 0.0),
                new Translation2d(36.0, 24.0),
                new Translation2d(60.0, 24.0));

        // Create the reference trajectory (straight line motion between waypoints).
        Trajectory<Translation2d> trajectory = new Trajectory<>(waypoints);
        final DistanceView<Translation2d> distance_view = new DistanceView<>(trajectory);

        Assert.assertEquals(0.0, distance_view.first_interpolant(), kTestEpsilon);
        Assert.assertEquals(84.0, distance_view.last_interpolant(), kTestEpsilon);

        Assert.assertEquals(waypoints.get(0), distance_view.sample(0.0).state());
        Assert.assertEquals(waypoints.get(0).interpolate(waypoints.get(1), 0.5), distance_view.sample(12.0).state());
        Assert.assertEquals(waypoints.get(3).interpolate(waypoints.get(4), 0.5), distance_view.sample(72.0).state());
    }

}
