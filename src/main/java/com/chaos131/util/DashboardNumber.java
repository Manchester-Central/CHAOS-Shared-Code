// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A tool for creating numbers that can be easily used in the code and also updated on a dashboard
 */
public class DashboardNumber {
  /** List of all values used over time, to be easier to revisit values */
  private static Set<String> ChangedValues = new HashSet<>();

  /** Shared list of all updaters that can be iterated over */
  private static List<DashboardNumber> AllUpdaters = new ArrayList<>();

  /** The NetworkNumber that is used for getting updates to/from the NetworkTables. */
  protected LoggedNetworkNumber m_networkNumber;

  /** Value being stored */
  private double m_value;

  /** Name of the value in network tables */
  private String m_name;

  /** What to do when the number is changed */
  private Consumer<Double> m_onUpdate;

  /** Adds a task to periodically check all dashboard numbers */
  static {
    PeriodicTasks.getInstance().addTask(DashboardNumber::checkAll);
  }

  /**
   * Creates a value that can be updated via NetworkTables.
   *
   * @param name of the field in network tables
   * @param startValue the initial value to be used
   */
  public DashboardNumber(String name, double startValue) {
    this(name, startValue, false, (value) -> {});
  }

  /**
   * Creates a value that can be updated via NetworkTables. Note: `onUpdate` will NOT be called
   * immediately with `initialValue`.
   *
   * @param name of the field in network tables
   * @param startValue the initial value to be used
   * @param onUpdate the consumer that will be called when the value is updated
   */
  public DashboardNumber(String name, double startValue, Consumer<Double> onUpdate) {
    this(name, startValue, false, onUpdate);
  }

  /**
   * Creates a value that can be updated via NetworkTables
   *
   * @param name of the field in network tables
   * @param startValue the initial value to be used
   * @param willTriggerWithInitialValue if true, `onUpdate` will be immediately called with
   *     `initialValue`
   * @param onUpdate the consumer that will be called when the value is updated
   */
  public DashboardNumber(
      String name,
      double startValue,
      boolean willTriggerWithInitialValue,
      Consumer<Double> onUpdate) {
    m_value = startValue;
    m_name = name;
    m_onUpdate = onUpdate;
    if (willTriggerWithInitialValue) {
      onUpdate.accept(m_value);
    }
    m_networkNumber = new LoggedNetworkNumber("DashboardNumbers/" + name, startValue);
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
    var newValue = m_networkNumber.get();
    if (newValue != m_value) {
      m_value = newValue;
      m_onUpdate.accept(m_value);
      ChangedValues.add(m_name);
    }
  }

  /** Checks every Dashboard number and updates them if they need to be updated */
  private static void checkAll() {
    for (DashboardNumber dashboardNumber : AllUpdaters) {
      dashboardNumber.checkValue();
    }
    Logger.recordOutput("DashboardNumbers/ChangedValues", ChangedValues.toArray(new String[0]));
  }
}
