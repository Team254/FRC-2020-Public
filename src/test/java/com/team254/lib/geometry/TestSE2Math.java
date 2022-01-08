package com.team254.lib.geometry;

import com.team254.lib.util.Util;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class TestSE2Math {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void testRotation2d() {
        // Test constructors
        Rotation2d rot1 = new Rotation2d();
        Assert.assertEquals(1, rot1.cos(), kTestEpsilon);
        Assert.assertEquals(0, rot1.sin(), kTestEpsilon);
        Assert.assertEquals(0, rot1.tan(), kTestEpsilon);
        Assert.assertEquals(0, rot1.getDegrees(), kTestEpsilon);
        Assert.assertEquals(0, rot1.getRadians(), kTestEpsilon);

        rot1 = new Rotation2d(1, 1, true);
        Assert.assertEquals(Math.sqrt(2) / 2, rot1.cos(), kTestEpsilon);
        Assert.assertEquals(Math.sqrt(2) / 2, rot1.sin(), kTestEpsilon);
        Assert.assertEquals(1, rot1.tan(), kTestEpsilon);
        Assert.assertEquals(45, rot1.getDegrees(), kTestEpsilon);
        Assert.assertEquals(Math.PI / 4, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromRadians(Math.PI / 2);
        Assert.assertEquals(0, rot1.cos(), kTestEpsilon);
        Assert.assertEquals(1, rot1.sin(), kTestEpsilon);
        Assert.assertTrue(1 / kTestEpsilon < rot1.tan());
        Assert.assertEquals(90, rot1.getDegrees(), kTestEpsilon);
        Assert.assertEquals(Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(270);
        Assert.assertEquals(0, rot1.cos(), kTestEpsilon);
        Assert.assertEquals(-1, rot1.sin(), kTestEpsilon);
        System.out.println(rot1.tan());
        Assert.assertTrue(-1 / kTestEpsilon > rot1.tan());
        Assert.assertEquals(-90, rot1.getDegrees(), kTestEpsilon);
        Assert.assertEquals(-Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        // Test inversion
        rot1 = Rotation2d.fromDegrees(270);
        Rotation2d rot2 = rot1.inverse();
        Assert.assertEquals(0, rot2.cos(), kTestEpsilon);
        Assert.assertEquals(1, rot2.sin(), kTestEpsilon);
        Assert.assertTrue(1 / kTestEpsilon < rot2.tan());
        Assert.assertEquals(90, rot2.getDegrees(), kTestEpsilon);
        Assert.assertEquals(Math.PI / 2, rot2.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(1);
        rot2 = rot1.inverse();
        Assert.assertEquals(rot1.cos(), rot2.cos(), kTestEpsilon);
        Assert.assertEquals(-rot1.sin(), rot2.sin(), kTestEpsilon);
        Assert.assertEquals(-1, rot2.getDegrees(), kTestEpsilon);

        // Test rotateBy
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        Rotation2d rot3 = rot1.rotateBy(rot2);
        Assert.assertEquals(0, rot3.cos(), kTestEpsilon);
        Assert.assertEquals(1, rot3.sin(), kTestEpsilon);
        Assert.assertTrue(1 / kTestEpsilon < rot3.tan());
        Assert.assertEquals(90, rot3.getDegrees(), kTestEpsilon);
        Assert.assertEquals(Math.PI / 2, rot3.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.rotateBy(rot2);
        Assert.assertEquals(1, rot3.cos(), kTestEpsilon);
        Assert.assertEquals(0, rot3.sin(), kTestEpsilon);
        Assert.assertEquals(0, rot3.tan(), kTestEpsilon);
        Assert.assertEquals(0, rot3.getDegrees(), kTestEpsilon);
        Assert.assertEquals(0, rot3.getRadians(), kTestEpsilon);

        // A rotation times its inverse should be the identity
        Rotation2d identity = new Rotation2d();
        rot1 = Rotation2d.fromDegrees(21.45);
        rot2 = rot1.rotateBy(rot1.inverse());
        Assert.assertEquals(identity.cos(), rot2.cos(), kTestEpsilon);
        Assert.assertEquals(identity.sin(), rot2.sin(), kTestEpsilon);
        Assert.assertEquals(identity.getDegrees(), rot2.getDegrees(), kTestEpsilon);

        // Test interpolation
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(135);
        rot3 = rot1.interpolate(rot2, .5);
        Assert.assertEquals(90, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(135);
        rot3 = rot1.interpolate(rot2, .75);
        Assert.assertEquals(112.5, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.interpolate(rot2, .5);
        Assert.assertEquals(0, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        rot3 = rot1.interpolate(rot2, .5);
        Assert.assertEquals(45, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        rot3 = rot1.interpolate(rot2, .5);
        Assert.assertEquals(45, rot3.getDegrees(), kTestEpsilon);

        // Test parallel.
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        Assert.assertTrue(rot1.isParallel(rot2));

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        Assert.assertFalse(rot1.isParallel(rot2));

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-135);
        Assert.assertTrue(rot1.isParallel(rot2));
    }

    @Test
    public void testTranslation2d() {
        // Test constructors
        Translation2d pos1 = new Translation2d();
        Assert.assertEquals(0, pos1.x(), kTestEpsilon);
        Assert.assertEquals(0, pos1.y(), kTestEpsilon);
        Assert.assertEquals(0, pos1.norm(), kTestEpsilon);

        pos1 = new Translation2d(3, 4);
        Assert.assertEquals(3, pos1.x(), kTestEpsilon);
        Assert.assertEquals(4, pos1.y(), kTestEpsilon);
        Assert.assertEquals(5, pos1.norm(), kTestEpsilon);

        // Test inversion
        pos1 = new Translation2d(3.152, 4.1666);
        Translation2d pos2 = pos1.inverse();
        Assert.assertEquals(-pos1.x(), pos2.x(), kTestEpsilon);
        Assert.assertEquals(-pos1.y(), pos2.y(), kTestEpsilon);
        Assert.assertEquals(pos1.norm(), pos2.norm(), kTestEpsilon);

        // Test rotateBy
        pos1 = new Translation2d(2, 0);
        Rotation2d rot1 = Rotation2d.fromDegrees(90);
        pos2 = pos1.rotateBy(rot1);
        Assert.assertEquals(0, pos2.x(), kTestEpsilon);
        Assert.assertEquals(2, pos2.y(), kTestEpsilon);
        Assert.assertEquals(pos1.norm(), pos2.norm(), kTestEpsilon);

        pos1 = new Translation2d(2, 0);
        rot1 = Rotation2d.fromDegrees(-45);
        pos2 = pos1.rotateBy(rot1);
        Assert.assertEquals(Math.sqrt(2), pos2.x(), kTestEpsilon);
        Assert.assertEquals(-Math.sqrt(2), pos2.y(), kTestEpsilon);
        Assert.assertEquals(pos1.norm(), pos2.norm(), kTestEpsilon);

        // Test translateBy
        pos1 = new Translation2d(2, 0);
        pos2 = new Translation2d(-2, 1);
        Translation2d pos3 = pos1.translateBy(pos2);
        Assert.assertEquals(0, pos3.x(), kTestEpsilon);
        Assert.assertEquals(1, pos3.y(), kTestEpsilon);
        Assert.assertEquals(1, pos3.norm(), kTestEpsilon);

        // A translation times its inverse should be the identity
        Translation2d identity = new Translation2d();
        pos1 = new Translation2d(2.16612, -23.55);
        pos2 = pos1.translateBy(pos1.inverse());
        Assert.assertEquals(identity.x(), pos2.x(), kTestEpsilon);
        Assert.assertEquals(identity.y(), pos2.y(), kTestEpsilon);
        Assert.assertEquals(identity.norm(), pos2.norm(), kTestEpsilon);

        // Test interpolation
        pos1 = new Translation2d(0, 1);
        pos2 = new Translation2d(10, -1);
        pos3 = pos1.interpolate(pos2, .5);
        Assert.assertEquals(5, pos3.x(), kTestEpsilon);
        Assert.assertEquals(0, pos3.y(), kTestEpsilon);

        pos1 = new Translation2d(0, 1);
        pos2 = new Translation2d(10, -1);
        pos3 = pos1.interpolate(pos2, .75);
        Assert.assertEquals(7.5, pos3.x(), kTestEpsilon);
        Assert.assertEquals(-.5, pos3.y(), kTestEpsilon);
    }

    @Test
    public void testPose2d() {
        // Test constructors
        Pose2d pose1 = new Pose2d();
        Assert.assertEquals(0, pose1.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(0, pose1.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(0, pose1.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(45));
        Assert.assertEquals(3, pose1.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(4, pose1.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(45, pose1.getRotation().getDegrees(), kTestEpsilon);

        // Test transformation
        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        Pose2d pose2 = new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(0));
        Pose2d pose3 = pose1.transformBy(pose2);
        Assert.assertEquals(3, pose3.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(5, pose3.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(90, pose3.getRotation().getDegrees(), kTestEpsilon);

        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(-90));
        pose3 = pose1.transformBy(pose2);
        Assert.assertEquals(3, pose3.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(5, pose3.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(0, pose3.getRotation().getDegrees(), kTestEpsilon);

        // A pose times its inverse should be the identity
        Pose2d identity = new Pose2d();
        pose1 = new Pose2d(new Translation2d(3.51512152, 4.23), Rotation2d.fromDegrees(91.6));
        pose2 = pose1.transformBy(pose1.inverse());
        Assert.assertEquals(identity.getTranslation().x(), pose2.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(identity.getTranslation().y(), pose2.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(identity.getRotation().getDegrees(), pose2.getRotation().getDegrees(), kTestEpsilon);

        // Test interpolation
        // Movement from pose1 to pose2 is along a circle with radius of 10 units
        // centered at (3, -6)
        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(13, -6), Rotation2d.fromDegrees(0.0));
        pose3 = pose1.interpolate(pose2, .5);
        double expected_angle_rads = Math.PI / 4;
        Assert.assertEquals(3.0 + 10.0 * Math.cos(expected_angle_rads), pose3.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(-6.0 + 10.0 * Math.sin(expected_angle_rads), pose3.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(expected_angle_rads, pose3.getRotation().getRadians(), kTestEpsilon);

        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(13, -6), Rotation2d.fromDegrees(0.0));
        pose3 = pose1.interpolate(pose2, .75);
        expected_angle_rads = Math.PI / 8;
        Assert.assertEquals(3.0 + 10.0 * Math.cos(expected_angle_rads), pose3.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(-6.0 + 10.0 * Math.sin(expected_angle_rads), pose3.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(expected_angle_rads, pose3.getRotation().getRadians(), kTestEpsilon);
    }

    @Test
    public void testTwist() {
        // Exponentiation (integrate twist to obtain a Pose2d)
        Twist2d twist = new Twist2d(1.0, 0.0, 0.0);
        Pose2d pose = Pose2d.exp(twist);
        Assert.assertEquals(1.0, pose.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(0.0, pose.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(0.0, pose.getRotation().getDegrees(), kTestEpsilon);

        // Scaled.
        twist = new Twist2d(1.0, 0.0, 0.0);
        pose = Pose2d.exp(twist.scaled(2.5));
        Assert.assertEquals(2.5, pose.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(0.0, pose.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(0.0, pose.getRotation().getDegrees(), kTestEpsilon);

        // Logarithm (find the twist to apply to obtain a given Pose2d)
        pose = new Pose2d(new Translation2d(2.0, 2.0), Rotation2d.fromRadians(Math.PI / 2));
        twist = Pose2d.log(pose);
        Assert.assertEquals(Math.PI, twist.dx, kTestEpsilon);
        Assert.assertEquals(0.0, twist.dy, kTestEpsilon);
        Assert.assertEquals(Math.PI / 2, twist.dtheta, kTestEpsilon);

        // Logarithm is the inverse of exponentiation.
        Pose2d new_pose = Pose2d.exp(twist);
        Assert.assertEquals(new_pose.getTranslation().x(), pose.getTranslation().x(), kTestEpsilon);
        Assert.assertEquals(new_pose.getTranslation().y(), pose.getTranslation().y(), kTestEpsilon);
        Assert.assertEquals(new_pose.getRotation().getDegrees(), pose.getRotation().getDegrees(), kTestEpsilon);
    }
}
