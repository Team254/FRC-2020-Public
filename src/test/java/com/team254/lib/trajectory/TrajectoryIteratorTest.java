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
public class TrajectoryIteratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2d> kWaypoints = Arrays.asList(
            new Translation2d(0.0, 0.0),
            new Translation2d(24.0, 0.0),
            new Translation2d(36.0, 12.0),
            new Translation2d(60.0, 12.0));

    @Test
    public void test() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);
        TrajectoryIterator<Translation2d> iterator = new TrajectoryIterator<>(traj.getIndexView());

        // Initial conditions.
        Assert.assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertEquals(kWaypoints.get(0), iterator.getState());
        Assert.assertFalse(iterator.isDone());

        // Advance forward.
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), iterator.preview(0.5).state());
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), iterator.advance(0.5).state());
        Assert.assertEquals(0.5, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(2.5, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertFalse(iterator.isDone());

        // Advance backwards.
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), iterator.preview(-0.25).state());
        Assert.assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), iterator.advance(-0.25).state());
        Assert.assertEquals(0.25, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(2.75, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertFalse(iterator.isDone());

        // Advance past end.
        Assert.assertEquals(kWaypoints.get(3), iterator.preview(5.0).state());
        Assert.assertEquals(kWaypoints.get(3), iterator.advance(5.0).state());
        Assert.assertEquals(3.0, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(0.0, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertTrue(iterator.isDone());

        // Advance past beginning.
        Assert.assertEquals(kWaypoints.get(0), iterator.preview(-5.0).state());
        Assert.assertEquals(kWaypoints.get(0), iterator.advance(-5.0).state());
        Assert.assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        Assert.assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        Assert.assertFalse(iterator.isDone());
    }

}
