package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TimedStateTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // At (0,0,0), t=0, v=0, acceleration=1
        final TimedState<Pose2d> start_state = new TimedState<>(Pose2d.fromTranslation(new Translation2d(0.0, 0.0)),
                0.0, 0.0, 1.0);

        // At (.5,0,0), t=1, v=1, acceleration=0
        final TimedState<Pose2d> end_state = new TimedState<>(Pose2d.fromTranslation(new Translation2d(0.5, 0.0)), 1.0,
                1.0, 0.0);

        Assert.assertEquals(start_state, start_state.interpolate(end_state, 0.0));
        Assert.assertEquals(end_state, start_state.interpolate(end_state, 1.0));
        Assert.assertEquals(end_state, end_state.interpolate(start_state, 0.0));
        System.out.println(end_state.interpolate(start_state, 1.0));
        Assert.assertEquals(start_state, end_state.interpolate(start_state, 1.0));

        final TimedState<Pose2d> intermediate_state = start_state.interpolate(end_state, 0.5);
        Assert.assertEquals(0.5, intermediate_state.t(), kTestEpsilon);
        Assert.assertEquals(start_state.acceleration(), intermediate_state.acceleration(), kTestEpsilon);
        Assert.assertEquals(0.5, intermediate_state.velocity(), kTestEpsilon);
        Assert.assertEquals(0.125, intermediate_state.state().getTranslation().x(), kTestEpsilon);
    }

}
