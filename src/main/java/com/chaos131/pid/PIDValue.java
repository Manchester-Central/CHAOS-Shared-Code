// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.pid;

/**
 * A PID update value.
 *
 * @deprecated We use {@link com.chaos131.ctre.ChaosTalonFxTuner} for CTRE PID tuning and WPILib's
 *     PIDController already supports dashboard tuning.
 */
@Deprecated(since = "2026.1", forRemoval = true)
public class PIDValue {
  public final double P, I, D;

  public PIDValue(double p, double i, double d) {
    P = p;
    I = i;
    D = d;
  }
}
