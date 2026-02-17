// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.tables;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public interface ITableData<U extends Unit, M extends Measure<U>> {
  M getMeasure();

  ITableData<U, M> mergeData(
      Distance targetMeasure, ITableData<U, M> data1, ITableData<U, M> data2);
}
