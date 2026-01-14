// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.function.BiConsumer;

/** This creates a class to easily tune TalonFXConfigs for 1 or more motors. */
public class ChaosTalonFxTuner {
  private String m_name;
  private ChaosTalonFx[] m_talons;

  /**
   * Creates a tuner for modifying numeric values of TalonFxConfigs.
   *
   * @param name the name of the motor tuner
   * @param talons the list of talons to tune
   */
  public ChaosTalonFxTuner(String name, ChaosTalonFx... talons) {
    m_name = name;
    m_talons = talons;
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
      String valueName, double initialValue, BiConsumer<TalonFXConfiguration, Double> onUpdate) {
    return new DashboardNumber(
        "TalonFxConfig/" + m_name + "/" + valueName,
        initialValue,
        true,
        false,
        newValue -> {
          for (ChaosTalonFx chaosTalonFx : m_talons) {
            onUpdate.accept(chaosTalonFx.Configuration, newValue);
            chaosTalonFx.applyConfig();
          }
        });
  }
}
