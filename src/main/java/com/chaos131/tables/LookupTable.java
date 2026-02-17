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
public abstract class LookupTable<
    U extends Unit, M extends Measure<U>, TR extends ITableRow<U, M>> {
  private List<TR> m_data = new ArrayList<>();
  private Comparator<TR> m_comparator =
      new Comparator<TR>() {
        public int compare(TR row1, TR row2) {
          return row1.getMeasure().compareTo(row2.getMeasure());
        }
      };

  public LookupTable() {}

  public LookupTable<U, M, TR> addRow(TR row) {
    m_data.add(row);
    m_data.sort(m_comparator);
    return this;
  }

  public LookupTable<U, M, TR> addRows(List<TR> rows) {
    m_data.addAll(rows);
    m_data.sort(m_comparator);
    return this;
  }

  public TR performLookup(M measure) {
    var exactRow =
        m_data.stream().filter(row -> row.getMeasure().isEquivalent(measure)).findFirst();
    if (exactRow.isPresent()) {
      return exactRow.get();
    }

    TR firstEntry = m_data.get(0);
    if (measure.lt(firstEntry.getMeasure())) {
      return firstEntry;
    }

    TR lastEntry = m_data.get(m_data.size() - 1);
    if (measure.gt(lastEntry.getMeasure())) {
      return lastEntry;
    }

    var lowerRow =
        m_data.stream()
            .filter(row -> row.getMeasure().lte(measure))
            .findFirst()
            .get(); // TODO: check logic
    var lowerRowIndex = m_data.indexOf(lowerRow);
    var higherRow = m_data.get(lowerRowIndex + 1);
    return mergeRows(measure, lowerRow, higherRow);
  }

  protected double interpolate(
      M targetMeasure,
      ITableRow<U, M> row1,
      ITableRow<U, M> row2,
      Function<ITableRow<U, M>, Double> valueSupplier) {
    var x1 = row1.getMeasure().magnitude();
    var x2 = row2.getMeasure().magnitude();
    var y1 = valueSupplier.apply(row1);
    var y2 = valueSupplier.apply(row2);
    var x = targetMeasure.magnitude();
    return y1 + ((x - x1) * (y2 - y1)) / (x2 - x1); // TODO: validate formula
  }

  abstract TR mergeRows(M targetMeasure, TR row1, TR row2);
}
