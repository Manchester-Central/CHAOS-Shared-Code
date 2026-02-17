// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.tables;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public interface ITableRow<U extends Unit, M extends Measure<U>> {
  M getMeasure();

  ITableRow<U, M> mergeRow(
      M targetMeasure, ITableRow<U, M> otherRow);
}
