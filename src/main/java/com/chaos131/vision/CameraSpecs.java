package com.chaos131.vision;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/** Physical and calibration values for a camera system */
public class CameraSpecs {
  /** Image frame sizes */
  public int height, width;

  /** Field of view values in degrees */
  public Angle HFOV, VFOV;

  /** Deviation Coefficients */
  public double minimum_error, error_exponent, error_multiplier;

  /** Deviation Coefficients */
  public double distance_scalar, tag_count_scalar, robot_speed_scalar;

  /** Deviation Coefficients */
  public double confidence_requirement;

  /** Deviation Coefficients */
  public LinearVelocity max_speed_acceptable;

  public AngularVelocity max_rotation_acceptable;

  public Distance max_distance_acceptable;
}
