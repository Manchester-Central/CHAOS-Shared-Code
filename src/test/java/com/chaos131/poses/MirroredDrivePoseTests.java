package com.chaos131.poses;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MirroredDrivePoseTests {
  /** This is an example of a DrivePose class that can be implemented in your robot code */
  class DrivePoseImpl extends MirroredFieldPose {
    public static final Translation2d midpoint = new Translation2d(10, 4);
    public static final Alliance DefaultAlliance = Alliance.Blue;

    protected DrivePoseImpl(String name, Pose2d bluePose) {
      super(midpoint, DefaultAlliance, name, bluePose);
    }

    protected DrivePoseImpl(Pose2d bluePose) {
      super(midpoint, DefaultAlliance, null, bluePose);
    }

    @Override
    protected Alliance getCurrentAlliance() {
      // Override so that DriverStation is not called in test mode
      return Alliance.Blue;
    }
  }

  @BeforeEach
  public void clean() {
    DrivePoseImpl.FieldPoses.clear();
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

    assertEquals(
        DrivePoseImpl.getClosestKnownPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
        pose1.getBluePose());
    assertEquals(
        DrivePoseImpl.getClosestKnownPose(new Pose2d(5, 7, Rotation2d.fromDegrees(180))),
        pose2.getBluePose());
    assertEquals(
        DrivePoseImpl.getClosestKnownPose(new Pose2d(8, 0, Rotation2d.fromDegrees(180))),
        pose3.getBluePose());
  }

  // @Test
  // public void testAngleFrom() {
  //     var pose1 = new DrivePoseImpl(new Pose2d());
  //     var angle = pose1.angleFrom(new Translation2d(4, 0));
  //     assertEquals(angle.getRadians(), 0, epsilon);
  //     angle = pose1.angleFrom(new Translation2d(0, 5));
  //     assertEquals(angle.getRadians(), Math.PI/2, epsilon);

  //     var pose2 = new DrivePoseImpl(new Pose2d(2, 2, new Rotation2d()));
  //     angle = pose2.angleFrom(new Translation2d(4, 0));
  //     assertEquals(angle.getRadians(), -Math.PI/4, epsilon);
  //     angle = pose2.angleFrom(new Translation2d(3, 1));
  //     assertEquals(angle.getRadians(), -Math.PI/4, epsilon);
  //     angle = pose2.angleFrom(new Translation2d(3, 0.5));
  //     assertEquals(angle.getRadians(), 0.982794, epsilon);
  // }

  // @Test
  // public void testMakeWPIBluePose() {
  //     double field_width = 16;
  //     double field_height = 8;

  //     Matrix<N4, N1> coord = MatBuilder.fill(Nat.N4(), Nat.N1(), new double[] {
  //         0,0,0,0
  //     });
  //     var newpose = DrivePoseImpl.makeWPIBluePoseFromCenteredPose(coord,
  //         field_width, field_height, Alliance.Blue, "TestPose1");
  //     assertEquals(newpose.getBluePose().getTranslation().getX(), field_width/2, epsilon);
  //     assertEquals(newpose.getBluePose().getTranslation().getY(), field_height/2, epsilon);

  //     coord = MatBuilder.fill(Nat.N4(), Nat.N1(), new double[] {
  //         8,0,0,0
  //     });
  //     newpose = DrivePoseImpl.makeWPIBluePoseFromCenteredPose(coord,
  //         field_width, field_height, Alliance.Blue, "TestPose2");
  //     assertEquals(newpose.getBluePose().getTranslation().getX(), 8+field_width/2, epsilon);
  //     assertEquals(newpose.getBluePose().getTranslation().getY(), field_height/2, epsilon);

  //     coord = MatBuilder.fill(Nat.N4(), Nat.N1(), new double[] {
  //         2,2,0,0
  //     });
  //     newpose = DrivePoseImpl.makeWPIBluePoseFromCenteredPose(coord,
  //         field_width, field_height, Alliance.Blue, "TestPose3");
  //     assertEquals(newpose.getBluePose().getTranslation().getX(), 10, epsilon);
  //     assertEquals(newpose.getBluePose().getTranslation().getY(), 6, epsilon);
  // }
}
