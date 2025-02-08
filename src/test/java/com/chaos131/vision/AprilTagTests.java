package com.chaos131.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.chaos131.util.Quad;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
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
  public void testSingleTag() {
    double[] transform;
    try {
      transform = // 4x4 Transformation Matrix for Fmap2025 ID1
          new ObjectMapper()
              .readValue(
                  "[-0.5877852522924729,-0.8090169943749473,0,7.923198000000001,0.8090169943749473,-0.5877852522924729,0,-3.3706799999999997,0,0,1,1.4859,0,0,0,1]",
                  double[].class);
      var trans_mat = MatBuilder.fill(Nat.N4(), Nat.N4(), transform);
      AprilTag id1 = new AprilTag(1, trans_mat, 165.1 / 1000.0);
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
      AprilTag id2 = new AprilTag(2, trans_mat, 165.1 / 1000.0);
      assertEquals(id2.pose2d.getRotation().getDegrees(), 234 - 360, 1);
    } catch (JsonProcessingException e) {
      // Failed to convert string to matrix
      assertTrue(false);
    }
  }
}
