// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.function.Consumer;

/** A utility for creating dashboard synced values in terms of units */
public class DashboardUnit<U extends Unit, M extends Measure<U>> {

  /** The DashboardNumber used for getting updates to/from the NetworkTables. */
  protected DashboardNumber m_dsNumber;

  /**
   * The default unit the DashboardUnit was created with. The Dashboard number with be displayed
   * with this unit.
   */
  protected U m_defaultUnit;

  /** What to do when the number is changed */
  protected Consumer<M> m_onUpdate;

  /**
   * Creates a measure that can be updated on the Dashboard
   *
   * @param name The name for this value on the Dashboard
   * @param defaultValue The default value to be displayed
   * @param willTriggerWithInitialValue if true, `onUpdate` will be immediately called with
   *     `initialValue`
   * @param onUpdate The consumer that will be called when the value is updated
   */
  public DashboardUnit(
      String name, M defaultValue, boolean willTriggerWithInitialValue, Consumer<M> onUpdate) {
    m_defaultUnit = defaultValue.unit();
    m_onUpdate = onUpdate;
    m_dsNumber =
        new DashboardNumber(
            name + "_" + m_defaultUnit.toString(),
            defaultValue.in(m_defaultUnit),
            willTriggerWithInitialValue,
            (value) -> m_onUpdate.accept(convertDoubleToMeasure(value)));
  }

  /**
   * Creates a measure that can be updated on the Dashboard
   *
   * @param name The name for this value on the Dashboard
   * @param defaultValue The default value to be displayed
   */
  public DashboardUnit(String name, M defaultValue) {
    this(name, defaultValue, false, (value) -> {});
  }

  /** Get the current value of the Measure. */
  public M get() {
    return convertDoubleToMeasure(m_dsNumber.get());
  }

  /**
   * Converts a double value into a measure with the default unit. Note: Java warnings are disabled
   * because the compiler cannot safely cast from Measure<?> to M, but we know it's safe.
   *
   * @param value The value to convert to a measure
   */
  @SuppressWarnings("unchecked")
  private M convertDoubleToMeasure(double value) {
    return (M) m_defaultUnit.of(value);
  }
}
