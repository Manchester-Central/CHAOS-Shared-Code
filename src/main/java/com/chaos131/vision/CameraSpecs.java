package com.chaos131.vision;

/** Physical and calibration values for a camera system */
public class CameraSpecs {
  /** Image frame sizes */
  public int height, width;

  /** Field of view values in degrees */
  public double HFOV, VFOV;

  /** Deviation Coefficients */
  public double minimum_error, error_exponent, error_multiplier;

  /** Deviation Coefficients */
  public double distance_scalar, tag_count_scalar, robot_speed_scalar;

  /** Deviation Coefficients */
  public double confidence_requirement;

  /** Deviation Coefficients */
  public double max_speed_acceptable, max_rotation_acceptable, max_distance_acceptable;
}
