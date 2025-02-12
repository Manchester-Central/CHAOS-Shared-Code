// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

/**
 * A tool for creating numbers that can be easily used in the code and also updated on a dashboard
 */
public class DashboardNumber {
  /** List of all values used over time, to be easier to revisit values */
  private static Set<String> ChangedValues = new HashSet<>();

  /** Shared list of all updaters that can be iterated over */
  private static List<DashboardNumber> AllUpdaters = new ArrayList<>();

  /** Value being stored */
  private double m_value;

  /** Name of the value in network tables */
  private String m_name;

  /** What to do when the number is changed */
  private Consumer<Double> m_onUpdate;

  /** checks if the tuning is enabled...? */
  private boolean m_tuningEnabled;

  /**
   * Creates a value that can be updated via NetworkTables. Note: `onUpdate` will be immediately
   * called with `initialValue`.
   *
   * @param name of the field in network tables
   * @param startValue the initial value to be used
   * @param tuningEnabled if tuning is enabled
   * @param onUpdate what to do when the value changes
   */
  public DashboardNumber(
      String name, double startValue, boolean tuningEnabled, Consumer<Double> onUpdate) {
    this(name, startValue, tuningEnabled, true, onUpdate);
  }

  /**
   * Creates a value that can be updated via NetworkTables
   *
   * @param name of the field in network tables
   * @param startValue the initial value to be used
   * @param tuningEnabled if tuning is enabled
   * @param willTriggerWithInitialValue if true, `onUpdate` will be immediately called with
   *     `initialValue`
   * @param onUpdate what to do when the value changes
   */
  public DashboardNumber(
      String name,
      double startValue,
      boolean tuningEnabled,
      boolean willTriggerWithInitialValue,
      Consumer<Double> onUpdate) {
    m_value = startValue;
    m_name = name;
    m_onUpdate = onUpdate;
    m_tuningEnabled = tuningEnabled;
    if (willTriggerWithInitialValue) {
      onUpdate.accept(m_value);
    }
    if (m_tuningEnabled) {
      SmartDashboard.putNumber(name, m_value);
    }
    AllUpdaters.add(this);
  }

  /**
   * @return the current value
   */
  public double get() {
    return m_value;
  }

  /** Check if the value is new, run the update function if it is */
  private void checkValue() {
    if (!m_tuningEnabled) {
      return;
    }
    var newValue = SmartDashboard.getNumber(m_name, m_value);
    if (newValue != m_value) {
      m_value = newValue;
      m_onUpdate.accept(m_value);
      ChangedValues.add(m_name);
    }
  }

  /** Checks every Dashboard number and updates them if they need to be updated */
  public static void checkAll() {
    for (DashboardNumber dashboardNumber : AllUpdaters) {
      dashboardNumber.checkValue();
    }
    SmartDashboard.putStringArray("ChangedValues", ChangedValues.toArray(new String[0]));
  }
}
