// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.pid;

/**
 * A PIDF update value.
 * @deprecated We use {@link com.chaos131.ctre.ChaosTalonFxTuner} for CTRE PID tuning and WPILib's PIDController already supports dashboard tuning.
 */
@Deprecated(since = "2026.1", forRemoval = true)
public class PIDFValue {
  public final double P, I, D, F;

  public PIDFValue(double p, double i, double d, double f) {
    P = p;
    I = i;
    D = d;
    F = f;
  }
}
