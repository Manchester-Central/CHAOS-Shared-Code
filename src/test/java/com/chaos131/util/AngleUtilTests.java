package com.chaos131.util;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class AngleUtilTests {

  @Test
  public void testIsNearWrapped() {
    var tolerance = Degrees.of(5);

    // Test close to 0
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(0), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(1), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(-1), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(5), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(-5), tolerance));

    // Test not close to 0
    assertTrue(!AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(6), tolerance));
    assertTrue(!AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(-6), tolerance));

    // Test close to 180
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(179), Degrees.of(180), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(-179), Degrees.of(180), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(181), Degrees.of(180), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(-181), Degrees.of(180), tolerance));

    // Test wrap-around
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(0), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(360), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(-360), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(720), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(-720), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(3600), tolerance));
    assertTrue(AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(-3600), tolerance));

    // Test clearly no results
    assertTrue(!AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(189), tolerance));
    assertTrue(!AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(131), tolerance));
    assertTrue(!AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(8), tolerance));
    assertTrue(!AngleUtil.isNearWrapped(Degrees.of(0), Degrees.of(1000), tolerance));
  }
}
