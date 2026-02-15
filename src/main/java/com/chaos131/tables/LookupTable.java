// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.tables;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/** Add your docs here. */
public class LookupTable<U extends Unit, M extends Measure<U>, TD extends ITableData<M>> {
    private List<TD> m_data = new ArrayList<>();

    public LookupTable() {}

    public LookupTable<M, TD> AddData(TD data) {
        m_data.add(data);
        return this;
    }

    public TD GetInterpolatedData() {

    }

    public TD GetLowerData() {

    }

    public TD GetHigherData() {
        
    }
}
