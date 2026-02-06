// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.ctre;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

/** This creates a class to easily tune CANcoderConfiguration for 1 or more CANcoders. */
public class ChaosCanCoderTuner {
  private String m_name;
  private ChaosCanCoder m_canCoder;
  private List<DashboardNumber> m_tunables =
      new ArrayList<>(); // Keep in list to prevent any garbage collection

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
   * Creates tunables for the MagnetSensorConfigs number values
   *
   * @param initialConfig the starting MagnetSensorConfigs values
   */
  public void tunableMagnetSensor(MagnetSensorConfigs initialConfig) {
    tunable(
        "DiscontinuityPoint_rotations",
        initialConfig.AbsoluteSensorDiscontinuityPoint,
        (config, newValue) -> config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = newValue);
    tunable(
        "Offset_rotations",
        initialConfig.MagnetOffset,
        (config, newValue) -> config.MagnetSensor.MagnetOffset = newValue);
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
    DashboardNumber dsNumber =
        new DashboardNumber(
            "CANCoderConfig/" + m_name + "/" + valueName,
            initialValue,
            true,
            false,
            newValue -> {
              onUpdate.accept(m_canCoder.Configuration, newValue);
              m_canCoder.applyConfig();
            });
    m_tunables.add(dsNumber);
    return dsNumber;
  }
}
