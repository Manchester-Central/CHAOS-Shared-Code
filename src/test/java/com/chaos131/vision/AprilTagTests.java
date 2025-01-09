package com.chaos131.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.chaos131.util.Quad;
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
}
