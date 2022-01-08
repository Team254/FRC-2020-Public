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
public class TrajectoryTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2d> kWaypoints = Arrays.asList(
            new Translation2d(0.0, 0.0),
            new Translation2d(24.0, 0.0),
            new Translation2d(36.0, 12.0),
            new Translation2d(60.0, 12.0));

    @Test
    public void testConstruction() {
        // Empty constructor.
        Trajectory<Translation2d> traj = new Trajectory<>();
        Assert.assertTrue(traj.isEmpty());
        Assert.assertEquals(0.0, traj.getIndexView().first_interpolant(), kTestEpsilon);
        Assert.assertEquals(0.0, traj.getIndexView().last_interpolant(), kTestEpsilon);
        Assert.assertEquals(0, traj.length());

        // Set states at construction time.
        traj = new Trajectory<>(kWaypoints);
        Assert.assertFalse(traj.isEmpty());
        Assert.assertEquals(0.0, traj.getIndexView().first_interpolant(), kTestEpsilon);
        Assert.assertEquals(3.0, traj.getIndexView().last_interpolant(), kTestEpsilon);
        Assert.assertEquals(4, traj.length());
    }

    @Test
    public void testStateAccessors() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);

        Assert.assertEquals(kWaypoints.get(0), traj.getState(0));
        Assert.assertEquals(kWaypoints.get(1), traj.getState(1));
        Assert.assertEquals(kWaypoints.get(2), traj.getState(2));
        Assert.assertEquals(kWaypoints.get(3), traj.getState(3));

        Assert.assertEquals(kWaypoints.get(0), traj.getInterpolated(0.0).state());
        Assert.assertEquals(traj.getInterpolated(0.0).index_floor(), 0);
        Assert.assertEquals(traj.getInterpolated(0.0).index_ceil(), 0);
        Assert.assertEquals(kWaypoints.get(1), traj.getInterpolated(1.0).state());
        Assert.assertEquals(traj.getInterpolated(1.0).index_floor(), 1);
        Assert.assertEquals(traj.getInterpolated(1.0).index_ceil(), 1);
        Assert.assertEquals(kWaypoints.get(2), traj.getInterpolated(2.0).state());
        Assert.assertEquals(traj.getInterpolated(2.0).index_floor(), 2);
        Assert.assertEquals(traj.getInterpolated(2.0).index_ceil(), 2);
        Assert.assertEquals(kWaypoints.get(3), traj.getInterpolated(3.0).state());
        Assert.assertEquals(traj.getInterpolated(3.0).index_floor(), 3);
        Assert.assertEquals(traj.getInterpolated(3.0).index_ceil(), 3);

        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), .25), traj.getInterpolated(0.25).state());
        Assert.assertEquals(traj.getInterpolated(0.25).index_floor(), 0);
        Assert.assertEquals(traj.getInterpolated(0.25).index_ceil(), 1);
        Assert.assertEquals(kWaypoints.get(1).interpolate(kWaypoints.get(2), .5), traj.getInterpolated(1.5).state());
        Assert.assertEquals(traj.getInterpolated(1.5).index_floor(), 1);
        Assert.assertEquals(traj.getInterpolated(1.5).index_ceil(), 2);
        Assert.assertEquals(kWaypoints.get(2).interpolate(kWaypoints.get(3), .75), traj.getInterpolated(2.75).state());
        Assert.assertEquals(traj.getInterpolated(2.75).index_floor(), 2);
        Assert.assertEquals(traj.getInterpolated(2.75).index_ceil(), 3);

        Trajectory<Translation2d>.IndexView index_view = traj.getIndexView();
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), .25), index_view.sample(0.25).state());
        Assert.assertEquals(kWaypoints.get(1).interpolate(kWaypoints.get(2), .5), index_view.sample(1.5).state());
        Assert.assertEquals(kWaypoints.get(2).interpolate(kWaypoints.get(3), .75), index_view.sample(2.75).state());
    }
}
