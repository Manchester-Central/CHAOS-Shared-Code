// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.ctre;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

/** This creates a class to easily tune TalonFXConfigs for 1 or more motors. */
public class ChaosTalonFxTuner {
  private String m_name;
  private ChaosTalonFx[] m_talons;
  private List<DashboardNumber> m_tunables = new ArrayList<>(); // Keep in list to prevent any garbage collection

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
  public void tunableSlot0(Slot0Configs initialConfig) {
    tunable("Slot0/kP", initialConfig.kP, (config, newValue) -> config.Slot0.kP = newValue);
    tunable("Slot0/kI", initialConfig.kI, (config, newValue) -> config.Slot0.kI = newValue);
    tunable("Slot0/kD", initialConfig.kD, (config, newValue) -> config.Slot0.kD = newValue);
    tunable("Slot0/kG", initialConfig.kG, (config, newValue) -> config.Slot0.kG = newValue);
    tunable("Slot0/kS", initialConfig.kS, (config, newValue) -> config.Slot0.kS = newValue);
    tunable("Slot0/kV", initialConfig.kV, (config, newValue) -> config.Slot0.kV = newValue);
    tunable("Slot0/kA", initialConfig.kA, (config, newValue) -> config.Slot0.kA = newValue);
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
    DashboardNumber dsNumber = new DashboardNumber(
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
    m_tunables.add(dsNumber);
    return dsNumber;
  }
}
