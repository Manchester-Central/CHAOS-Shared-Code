// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.ctre;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Values needed to sim a CTRE motor for a subsystem/mechanism */
public record CtreMotorSimValues(
    DCMotorSim dcMotorSim,
    double gearRatio,
    boolean isMainSimMotor,
    ChassisReference orientation,
    MotorType motorType) {

  public static ChassisReference chassisReferenceFromInvertedValue(InvertedValue value) {
    return value == InvertedValue.CounterClockwise_Positive
        ? ChassisReference.CounterClockwise_Positive
        : ChassisReference.Clockwise_Positive;
  }
}
