// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

/** A TalonFX wrapper with automatic simulation support and helper functions. */
public class ChaosCanCoder extends CANcoder {
  public final CANcoderConfiguration Configuration = new CANcoderConfiguration();

  /** Creates the new TalonFX wrapper WITHOUT simulation support. */
  public ChaosCanCoder(int canId, String canBus) {
    super(canId, canBus);
  }

  /** Applies/burns the configuration to the motor. */
  public void applyConfig() {
    getConfigurator().apply(Configuration);
  }
}
