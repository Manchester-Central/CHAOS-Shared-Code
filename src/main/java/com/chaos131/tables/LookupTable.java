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

/**
 * A class that allows for looking up table rows (while allowing merging rows when inbetween known
 * values).
 */
public abstract class LookupTable<
    U extends Unit, M extends Measure<U>, TR extends ITableRow<U, M>> {
  private List<TR> m_data;
  private Comparator<TR> m_comparator =
      new Comparator<TR>() {
        public int compare(TR row1, TR row2) {
          return row1.getMeasure().compareTo(row2.getMeasure());
        }
      };

  public LookupTable() {
    m_data = new ArrayList<>();
  }

  public LookupTable(List<TR> rows) {
    m_data = new ArrayList<>(rows);
  }
  
  /**
   * Adds a row to the lookup table
   *
   * @param row the row to add
   */
  public LookupTable<U, M, TR> addRow(TR row) {
    m_data.add(row);
    m_data.sort(m_comparator);
    return this;
  }

  /**
   * Adds a row to the lookup table
   *
   * @param rows the rows to add
   */
  public LookupTable<U, M, TR> addRows(List<TR> rows) {
    m_data.addAll(rows);
    m_data.sort(m_comparator);
    return this;
  }

  /**
   * Looks up a table row given the measure. If an exact value is matched, the exact value is
   * returned. If below known values, the lowest value is returned. If above known values, the
   * highest value is returned. If between values, the result of mergeRows is returned
   *
   * @param measure the value to look up by
   */
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

    var highRow =
        m_data.stream()
            .filter(row -> row.getMeasure().gt(measure))
            .findFirst()
            .get(); // TODO: check logic
    var highRowIndex = m_data.indexOf(highRow);
    var lowRow = m_data.get(highRowIndex - 1);
    return mergeRows(measure, lowRow, highRow);
  }

  /**
   * Interpolates between two rows
   *
   * @param targetMeasure the target value to interpolate between measures
   * @param row1 one row
   * @param row2 the other row
   * @param valueSupplier a function to return the dependent variable to interpolate
   * @return The interpolates value
   */
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

  /**
   * Creates a new table row between the 2 rows
   *
   * @param targetMeasure the current measure to make a row for
   * @param row1 one row
   * @param row2 the other row
   */
  public abstract TR mergeRows(M targetMeasure, TR row1, TR row2);
}
