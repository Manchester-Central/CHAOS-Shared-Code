// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.tables;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/** A class to represent a table row for a lookup table */
public interface ITableRow<U extends Unit, M extends Measure<U>> {
  /** The measure to use as the independent variable for lookups */
  M getMeasure();
}
