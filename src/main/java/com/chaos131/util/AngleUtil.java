// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

public class AngleUtil {
  /**
   * Checks to see if 2 angles are close (handles -179 & 179)
   *
   * @param angle1 the first angle
   * @param angle2 the second angle
   * @param tolerance the angle difference accepted
   * @returntrue if angle1 and angle2's difference is within the tolerance
   */
  public static boolean isNearWrapped(Angle angle1, Angle angle2, Angle tolerance) {
    var angleDiffRads = MathUtil.angleModulus(angle1.minus(angle2).in(Radians));
    return Math.abs(angleDiffRads) <= tolerance.in(Radians);
  }
}
