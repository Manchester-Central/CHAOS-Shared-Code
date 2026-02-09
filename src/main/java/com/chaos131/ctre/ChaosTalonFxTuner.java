// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.ctre;

import com.chaos131.util.DashboardNumber;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

/** This creates a class to easily tune TalonFXConfigs for 1 or more motors. */
public class ChaosTalonFxTuner {
  private String m_name;
  private ChaosTalonFx[] m_talons;
  private List<DashboardNumber> m_tunables =
      new ArrayList<>(); // Keep in list to prevent any garbage collection

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

  /** Creates tunables for the Slot0 number values using the first motor's config */
  public ChaosTalonFxTuner withSlot0() {
    return withSlot0(m_talons[0].Configuration.Slot0);
  }

  /**
   * Creates tunables for the Slot0 number values
   *
   * @param initialConfig the starting Slot0 values
   */
  public ChaosTalonFxTuner withSlot0(Slot0Configs initialConfig) {
    tunable("Slot0_kP", initialConfig.kP, (config, newValue) -> config.Slot0.kP = newValue);
    tunable("Slot0_kI", initialConfig.kI, (config, newValue) -> config.Slot0.kI = newValue);
    tunable("Slot0_kD", initialConfig.kD, (config, newValue) -> config.Slot0.kD = newValue);
    tunable("Slot0_kG", initialConfig.kG, (config, newValue) -> config.Slot0.kG = newValue);
    tunable("Slot0_kS", initialConfig.kS, (config, newValue) -> config.Slot0.kS = newValue);
    tunable("Slot0_kV", initialConfig.kV, (config, newValue) -> config.Slot0.kV = newValue);
    tunable("Slot0_kA", initialConfig.kA, (config, newValue) -> config.Slot0.kA = newValue);
    return this;
  }

  /** Creates tunables for the CurrentLimitsConfigs number values using the first motor's config */
  public ChaosTalonFxTuner withCurrentLimits() {
    return withCurrentLimits(m_talons[0].Configuration.CurrentLimits);
  }

  /**
   * Creates tunables for the CurrentLimitsConfigs number values
   *
   * @param initialConfig the starting CurrentLimitsConfigs values
   */
  public ChaosTalonFxTuner withCurrentLimits(CurrentLimitsConfigs initialConfig) {
    tunable(
        "CurrentLimit_Stator",
        initialConfig.StatorCurrentLimit,
        (config, newValue) -> config.CurrentLimits.StatorCurrentLimit = newValue);
    tunable(
        "CurrentLimit_Supply",
        initialConfig.SupplyCurrentLimit,
        (config, newValue) -> config.CurrentLimits.SupplyCurrentLimit = newValue);
    tunable(
        "CurrentLimit_SupplyLower_Limit",
        initialConfig.SupplyCurrentLowerLimit,
        (config, newValue) -> config.CurrentLimits.SupplyCurrentLowerLimit = newValue);
    tunable(
        "CurrentLimit_SupplyLower_Time",
        initialConfig.SupplyCurrentLowerTime,
        (config, newValue) -> config.CurrentLimits.SupplyCurrentLowerTime = newValue);
    return this;
  }

  /** Creates tunables for the FeedbackConfigs number values using the first motor's config */
  public ChaosTalonFxTuner withFeedbackValues() {
    return withFeedbackValues(m_talons[0].Configuration.Feedback);
  }

  /**
   * Creates tunables for the FeedbackConfigs number values
   *
   * @param initialConfig the starting FeedbackConfigs values
   */
  public ChaosTalonFxTuner withFeedbackValues(FeedbackConfigs initialConfig) {
    tunable(
        "Feedback_RotorToSensorRatio",
        initialConfig.RotorToSensorRatio,
        (config, newValue) -> config.Feedback.RotorToSensorRatio = newValue);
    tunable(
        "Feedback_SensorToMechanismRatio",
        initialConfig.SensorToMechanismRatio,
        (config, newValue) -> config.Feedback.SensorToMechanismRatio = newValue);
    return this;
  }

  /**
   * Adds all tunable values (currently Slot0, CurrentLimits, and FeedbackValues) using the first
   * motor's config
   */
  public ChaosTalonFxTuner withAllConfigs() {
    return withAllConfigs(m_talons[0].Configuration);
  }

  /**
   * Adds all tunable values (currently Slot0, CurrentLimits, and FeedbackValues)
   *
   * @param initialConfig the starting config
   */
  public ChaosTalonFxTuner withAllConfigs(TalonFXConfiguration initialConfig) {
    withSlot0(initialConfig.Slot0);
    withCurrentLimits(initialConfig.CurrentLimits);
    withFeedbackValues(initialConfig.Feedback);
    return this;
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
    DashboardNumber dsNumber =
        new DashboardNumber(
            m_name + "/" + valueName,
            initialValue,
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
