// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.gamepads;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;


/** A CHAOS Wrapper around `CommandXboxController` */
public class Gamepad extends CommandXboxController {


    public Gamepad(int port) {
        super(port);
    }

    private double applyDeadband(double value) {
        return MathUtil.applyDeadband(value, 0.05);
    }

    @Override
    public double getLeftX() {
        return applyDeadband(super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return applyDeadband(-super.getLeftY());
    }

    @Override
    public double getRightX() {
        return applyDeadband(super.getRightX());
    }

    @Override
    public double getRightY() {
        return applyDeadband(-super.getRightY());
    }

    @Override
    public double getLeftTriggerAxis() {
        return applyDeadband(super.getLeftTriggerAxis());
    }

    @Override
    public double getRightTriggerAxis() {
        return applyDeadband(super.getRightTriggerAxis());
    }

    public double getLeftAngle() {
        return Math.atan2(getLeftY(), getLeftX());
    }

    public double getRightAngle() {
        return Math.atan2(getRightY(), getRightX());
    }

    public double getRightMagnitude() {
        return Math.hypot(getRightX(), getRightY());
    }

    public double getLeftMagnitude() {
        return Math.hypot(getLeftX(), getLeftY());
    }

}
