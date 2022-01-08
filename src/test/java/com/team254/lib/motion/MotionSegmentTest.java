package com.team254.lib.motion;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class MotionSegmentTest {

    @Test
    public void valid() {
        MotionSegment a = new MotionSegment(new MotionState(0.0, 0.0, 0.0, 0.0), new MotionState(0.0, 0.0, 0.0, 1.0));
        assertFalse(a.isValid());

        a = new MotionSegment(new MotionState(0.0, 0.0, 0.0, 1.0), new MotionState(1.0, 0.0, 1.0, 1.0));
        assertFalse(a.isValid());

        a = new MotionSegment(new MotionState(0.0, 0.0, 0.0, 1.0), new MotionState(1.0, .5, 1.0, 1.0));
        assertTrue(a.isValid());
    }

}
