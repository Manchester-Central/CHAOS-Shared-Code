// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.pid;

public class PIDValue {
    public final double P, I, D;

    public PIDValue(double p, double i, double d) {
        P = p;
        I = i;
        D = d;
    }
}
