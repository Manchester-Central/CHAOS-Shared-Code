// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import java.util.function.BiConsumer;

/** This creates a class to easily tune TalonFXConfigs for 1 or more motors. */
public class ChaosCanCoderTuner {
  private String m_name;
  private ChaosCanCoder m_canCoder;

  /**
   * Creates a tuner for modifying numeric values of TalonFxConfigs.
   *
   * @param name the name of the motor tuner
   * @param canCoder the list of talons to tune
   */
  public ChaosCanCoderTuner(String name, ChaosCanCoder canCoder) {
    m_name = name;
    m_canCoder = canCoder;
  }

  /**
   * Creates a tunable value for the TalonFxConfiguration and will apply/burn the value to the motor
   * when it changes.
   *
   * @param valueName the name of the value (e.g., "SupplyCurrentLimit")
   * @param initialValue the value to start at (is not applied by default)
   * @param onUpdate the function to update the configuration
   * @return the Dashboard number
   */
  public DashboardNumber tunable(
      String valueName, double initialValue, BiConsumer<CANcoderConfiguration, Double> onUpdate) {
    return new DashboardNumber(
        "CANCoderConfig/" + m_name + "/" + valueName,
        initialValue,
        true,
        false,
        newValue -> {
          onUpdate.accept(m_canCoder.Configuration, newValue);
          m_canCoder.applyConfig();
        });
  }
}
