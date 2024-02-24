package com.chaos131.poses;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MirroredDrivePoseTests {

    /** 
     * This is an example of a DrivePose class that can be implemented in your robot code
     */
    class DrivePoseImpl extends MirroredDrivePose {
        public static final double FieldWidthMeters = 10;
        public static final Alliance DefaultAlliance = Alliance.Blue;

        protected DrivePoseImpl(String name, Pose2d bluePose) {
            super(FieldWidthMeters, DefaultAlliance, name, bluePose);
        }

        protected DrivePoseImpl(Pose2d bluePose) {
            super(FieldWidthMeters, DefaultAlliance, null, bluePose);
        }

        @Override
        protected Alliance getCurrentAlliance() {
            // Override so that DriverStation is not called in test mode
            return Alliance.Blue;
        }
    }

    @BeforeEach
    public void clean() {
        DrivePoseImpl.DrivePoses.clear();
    }

    @Test
    public void testMirroringX() {
        var drivePose0Meters = new DrivePoseImpl(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        assertEquals(drivePose0Meters.getRedPose().getX(), 10);

        var drivePose3Meters = new DrivePoseImpl(new Pose2d(3, 0, Rotation2d.fromDegrees(0)));
        assertEquals(drivePose3Meters.getRedPose().getX(), 7);

        var drivePose5Meters = new DrivePoseImpl(new Pose2d(5, 0, Rotation2d.fromDegrees(0)));
        assertEquals(drivePose5Meters.getRedPose().getX(), 5);
    }

    @Test
    public void testMirroringY() {
        var drivePose0Meters = new DrivePoseImpl(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        assertEquals(drivePose0Meters.getRedPose().getY(), 0);

        var drivePose3Meters = new DrivePoseImpl(new Pose2d(0, 3, Rotation2d.fromDegrees(0)));
        assertEquals(drivePose3Meters.getRedPose().getY(), 3);

        var drivePose5Meters = new DrivePoseImpl(new Pose2d(0, 5, Rotation2d.fromDegrees(0)));
        assertEquals(drivePose5Meters.getRedPose().getY(), 5);
    }

    @Test
    public void testMirroringAngle() {
        var drivePose0Degrees = new DrivePoseImpl(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        assertTrue(drivePose0Degrees.getRedPose().getRotation().equals(Rotation2d.fromDegrees(180)));

        var drivePose90Degrees = new DrivePoseImpl(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
        assertTrue(drivePose90Degrees.getRedPose().getRotation().equals(Rotation2d.fromDegrees(90)));
        
        var drivePose180Degrees = new DrivePoseImpl(new Pose2d(0, 0, Rotation2d.fromDegrees(180)));
        assertTrue(drivePose180Degrees.getRedPose().getRotation().equals(Rotation2d.fromDegrees(0)));
        
        var drivePose270Degrees = new DrivePoseImpl(new Pose2d(0, 0, Rotation2d.fromDegrees(270)));
        assertTrue(drivePose270Degrees.getRedPose().getRotation().equals(Rotation2d.fromDegrees(270)));
    }

    @Test
    public void testGetClosestPose() {
        var pose1 = new DrivePoseImpl("pose1", new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        var pose2 = new DrivePoseImpl("pose2", new Pose2d(6, 6, Rotation2d.fromDegrees(0)));
        var pose3 = new DrivePoseImpl("pose3", new Pose2d(9, 0, Rotation2d.fromDegrees(0)));

        assertEquals(DrivePoseImpl.getClosestKnownPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180))), pose1.getBluePose());
        assertEquals(DrivePoseImpl.getClosestKnownPose(new Pose2d(5, 7, Rotation2d.fromDegrees(180))), pose2.getBluePose());
        assertEquals(DrivePoseImpl.getClosestKnownPose(new Pose2d(8, 0, Rotation2d.fromDegrees(180))), pose3.getBluePose());
    }
}
