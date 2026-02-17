// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.tables;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Function;

/** Add your docs here. */
public class LookupTable<U extends Unit, M extends Measure<U>, TD extends ITableData<U, M>> {
  private List<TD> m_data = new ArrayList<>();
  private Comparator<TD> m_comparator =
      new Comparator<TD>() {
        public int compare(TD o1, TD o2) {
          return o1.getMeasure().compareTo(o2.getMeasure());
        }
      };

  public LookupTable() {}

  public LookupTable<U, M, TD> addData(TD data) {
    m_data.add(data);
    m_data.sort(m_comparator);
    return this;
  }

  public LookupTable<U, M, TD> addData(List<TD> data) {
    m_data.addAll(data);
    m_data.sort(m_comparator);
    return this;
  }

  public TD performLookup(M measure) {
    var foundData =
        m_data.stream().filter(td -> td.getMeasure().lte(measure)).findFirst(); // TODO: check logic
    var lastEntry = m_data.get(m_data.size() - 1);
    // TODO: handle merging values when inbetween 2 options
    return foundData.orElse(lastEntry);
  }

  public static <U2 extends Unit, M2 extends Measure<U2>> double interpolate(
      M2 targetMeasure,
      ITableData<U2, M2> td1,
      ITableData<U2, M2> td2,
      Function<ITableData<U2, M2>, Double> valueSupplier) {
    var x1 = td1.getMeasure().magnitude();
    var x2 = td2.getMeasure().magnitude();
    var y1 = valueSupplier.apply(td1);
    var y2 = valueSupplier.apply(td2);
    var x = targetMeasure.magnitude();
    return y1 + ((x - x1) * (y2 - y1)) / (x2 - x1); // TODO: validate formula
  }
}
