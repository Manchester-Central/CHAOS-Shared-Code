// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.gamepads;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;


/** A CHAOS Wrapper around `CommandXboxController` */
public class Gamepad extends CommandXboxController {

    
    private SlewRateLimiter m_slewratelimiterLeftX;
    private SlewRateLimiter m_slewratelimiterLeftY;
    private SlewRateLimiter m_slewratelimiterRightX;
    private SlewRateLimiter m_slewratelimiterRightY;
   

    public Gamepad(int port, double value, double rotRate) {
        super(port);
        m_slewratelimiterLeftX = new SlewRateLimiter(value);
        m_slewratelimiterLeftY = new SlewRateLimiter(value);
        m_slewratelimiterRightX = new SlewRateLimiter(rotRate);
        m_slewratelimiterRightY = new SlewRateLimiter(rotRate);
    }

    

    private double applyDeadband(double value) {
        return MathUtil.applyDeadband(value, 0.05);
    }


    public double getSlewLeftX() {
        double newLeftX = m_slewratelimiterLeftX.calculate(super.getLeftX());
        return applyDeadband(newLeftX);
    }


    public double getSlewLeftY() {
        double newLeftY = m_slewratelimiterLeftY.calculate(-super.getLeftY());
        return applyDeadband(newLeftY);
    }

    @Override
    public double getLeftX() {
    m_slewratelimiterLeftX.calculate(super.getLeftX());
    return applyDeadband(super.getLeftX());
    }

    @Override
    public double getLeftY() {
    m_slewratelimiterLeftY.calculate(-super.getLeftY());
    return applyDeadband(-super.getLeftY());
    }

    @Override
    public double getRightX() {
        m_slewratelimiterRightX.calculate(super.getRightX());
        return applyDeadband(super.getRightX());
    }

    @Override
    public double getRightY() {
        m_slewratelimiterRightY.calculate(-super.getRightY());
        return applyDeadband(-super.getRightY());
    }

    public double getSlewRightX() {
        double newSlewRightX = m_slewratelimiterRightX.calculate(super.getRightX());
        return applyDeadband(newSlewRightX);
    }


    public double getSlewRightY() {
        double newSlewRightY = m_slewratelimiterRightY.calculate(-super.getRightY());
        return applyDeadband(newSlewRightY);
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
