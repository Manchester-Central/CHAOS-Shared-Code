package com.chaos131.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.chaos131.util.Quad;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

public class AprilTagTests {
  double DELTA = 0.001;

  @Test
  public void testQuads() {
    var q =
        new Quad(
            new Translation3d(3, 4, 0),
            new Translation3d(),
            new Translation3d(),
            new Translation3d());
    assertEquals(q.getPoints().size(), 4);

    assertEquals(q.ll.get(0), 3);
    assertEquals(q.ll.get(1), 4);
  }

  @Test
  public void testTagRotations() {
    double[] transform;
    Transform3d coord_shift = new Transform3d();
    try {
      transform = // 4x4 Transformation Matrix for Fmap2025 ID1
          new ObjectMapper()
              .readValue(
                  "[-0.5877852522924729,-0.8090169943749473,0,7.923198000000001,0.8090169943749473,-0.5877852522924729,0,-3.3706799999999997,0,0,1,1.4859,0,0,0,1]",
                  double[].class);
      var trans_mat = MatBuilder.fill(Nat.N4(), Nat.N4(), transform);
      AprilTag id1 = new AprilTag(1, trans_mat, 165.1 / 1000.0, coord_shift);
      assertEquals(id1.pose2d.getRotation().getDegrees(), 126, 1);
    } catch (JsonProcessingException e) {
      // Failed to convert string to matrix
      assertTrue(false);
    }

    try {
      transform = // 4x4 Transformation Matrix for Fmap2025 ID2
          new ObjectMapper()
              .readValue(
                  "[-0.5877852522924734,0.8090169943749473,0,7.923198000000001,-0.8090169943749473,-0.5877852522924734,0,3.3704799999999997,0,0,1,1.4859,0,0,0,1]",
                  double[].class);
      var trans_mat = MatBuilder.fill(Nat.N4(), Nat.N4(), transform);
      AprilTag id2 = new AprilTag(2, trans_mat, 165.1 / 1000.0, coord_shift);
      assertEquals(id2.pose2d.getRotation().getDegrees(), 234 - 360, 1);
    } catch (JsonProcessingException e) {
      // Failed to convert string to matrix
      assertTrue(false);
    }
  }

  @Test
  public void testTagCoordinateShift() {
    double[] transform;
    Transform3d coord_shift = new Transform3d(17.5482504 / 2, 8.0519016 / 2, 0, new Rotation3d());
    try {
      transform = // 4x4 Transformation Matrix for Fmap2025 ID1
          new ObjectMapper()
              .readValue(
                  "[-0.5877852522924729,-0.8090169943749473,0,7.923198000000001,0.8090169943749473,-0.5877852522924729,0,-3.3706799999999997,0,0,1,1.4859,0,0,0,1]",
                  double[].class);
      var trans_mat = MatBuilder.fill(Nat.N4(), Nat.N4(), transform);
      AprilTag id1 = new AprilTag(1, trans_mat, 165.1 / 1000.0, coord_shift);
      assertEquals(
          id1.pose2d.getTranslation().getDistance(new Translation2d(16.697, 0.6553)), 0, 0.01);
    } catch (JsonProcessingException e) {
      // Failed to convert string to matrix
      assertTrue(false);
    }
  }

  @Test
  public void testTagAxisAlignedDegree() {
    double[] transform;
    // These are values for FRC 2025's April Tag 17 and field dimensions
    Transform3d coord_shift = new Transform3d(17.5482504 / 2, 8.0519016 / 2, 0, new Rotation3d());
    try {
      transform = // 4x4 Transformation Matrix for Fmap2025 ID17 that faces straight right
          new ObjectMapper()
              .readValue(
                  "[1,0,0,-3.452953999999999,0,1,0,-0.00009999999999976694,0,0,1,0.308102,0,0,0,1]",
                  double[].class);
      var trans_mat = MatBuilder.fill(Nat.N4(), Nat.N4(), transform);
      AprilTag id21 = new AprilTag(21, trans_mat, 165.1 / 1000.0, coord_shift);
      assertEquals(id21.pose2d.getRotation().getDegrees(), 0, 1);
      // Translation2d values taken from Wolfram Alpha converting inches to meters, we check how far
      // off the two are
      assertEquals(
          id21.pose2d.getTranslation().getDistance(new Translation2d(5.321, 4.026)), 0, 0.01);
    } catch (JsonProcessingException e) {
      // Failed to convert string to matrix
      assertTrue(false);
    }

    try {
      transform = // 4x4 Transformation Matrix for Fmap2025 ID17 that faces straight right
          new ObjectMapper()
              .readValue(
                  "[-1,-1.2246467991473532e-16,0,-5.116399999999999,1.2246467991473532e-16,-1,0,-0.00009999999999976694,0,0,1,0.308102,0,0,0,1]",
                  double[].class);
      var trans_mat = MatBuilder.fill(Nat.N4(), Nat.N4(), transform);
      AprilTag id18 = new AprilTag(18, trans_mat, 165.1 / 1000.0, coord_shift);
      assertEquals(id18.pose2d.getRotation().getDegrees(), 180, 1);
      // Translation2d values taken from Wolfram Alpha converting inches to meters, we check how far
      // off the two are
      assertEquals(
          id18.pose2d.getTranslation().getDistance(new Translation2d(3.658, 4.026)), 0, 0.01);
    } catch (JsonProcessingException e) {
      // Failed to convert string to matrix
      assertTrue(false);
    }
  }
}
