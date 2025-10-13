// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A utility for creating dashboard synced values in terms of units
 */
public class DashboardUnit<U extends Unit, M extends Measure<U>> {

  protected LoggedNetworkNumber m_networkNumber;
  protected U m_defaultUnit;

  public DashboardUnit(String name, M defaultValue) {
    m_defaultUnit = defaultValue.unit();
    m_networkNumber =
        new LoggedNetworkNumber(
            name + "_" + m_defaultUnit.toString(), defaultValue.in(m_defaultUnit));
  }

  @SuppressWarnings("unchecked")
  public M get() {
    return (M) m_defaultUnit.of(m_networkNumber.get());
  }

}
