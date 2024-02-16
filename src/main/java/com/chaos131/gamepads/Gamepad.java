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
   

    public Gamepad(int port, double leftStickSlewRate, double rightStickSlewRate) {
        super(port);
        m_slewratelimiterLeftX = new SlewRateLimiter(leftStickSlewRate);
        m_slewratelimiterLeftY = new SlewRateLimiter(leftStickSlewRate);
        m_slewratelimiterRightX = new SlewRateLimiter(rightStickSlewRate);
        m_slewratelimiterRightY = new SlewRateLimiter(rightStickSlewRate);
    }

    public Gamepad(int port) {
        super(port);
    }

    private double applyDeadband(double value) {
        return MathUtil.applyDeadband(value, 0.05);
    }

    /**
     * @return Returns the deadbanded value of the joystick, and also attempts to update the slewrate calculator.
     */
    @Override
    public double getLeftX() {
        if (m_slewratelimiterLeftX != null)
            m_slewratelimiterLeftX.calculate(super.getLeftX());
        return applyDeadband(super.getLeftX());
    }

    /**
     * @return Returns the deadbanded value of the joystick, and also attempts to update the slewrate calculator.
     */
    @Override
    public double getLeftY() {
        if (m_slewratelimiterLeftY != null)
            m_slewratelimiterLeftY.calculate(-super.getLeftY());
        return applyDeadband(-super.getLeftY());
    }

    /**
     * @return Returns the deadbanded value of the joystick, and also attempts to update the slewrate calculator.
     */
    @Override
    public double getRightX() {
        if (m_slewratelimiterRightX != null)
            m_slewratelimiterRightX.calculate(super.getRightX());
        return applyDeadband(super.getRightX());
    }

    /**
     * @return Returns the deadbanded value of the joystick, and also attempts to update the slewrate calculator.
     */
    @Override
    public double getRightY() {
        if (m_slewratelimiterRightY != null)
            m_slewratelimiterRightY.calculate(-super.getRightY());
        return applyDeadband(-super.getRightY());
    }

    /**
     * @return Returns the slew rate derived value if there is a rate given, otherwise 0.
     */
    public double getSlewLeftX() {
        if (m_slewratelimiterLeftX == null)
            return 0;
        double newLeftX = m_slewratelimiterLeftX.calculate(super.getLeftX());
        return applyDeadband(newLeftX);
    }

    /**
     * @return Returns the slew rate derived value if there is a rate given, otherwise 0.
     */
    public double getSlewLeftY() {
        if (m_slewratelimiterLeftY == null)
            return 0;
        double newLeftY = m_slewratelimiterLeftY.calculate(-super.getLeftY());
        return applyDeadband(newLeftY);
    }

    /**
     * @return Returns the slew rate derived value if there is a rate given, otherwise 0.
     */
    public double getSlewRightX() {
        if (m_slewratelimiterRightX == null)
            return 0;
        double newSlewRightX = m_slewratelimiterRightX.calculate(super.getRightX());
        return applyDeadband(newSlewRightX);
    }

    /**
     * @return Returns the slew rate derived value if there is a rate given, otherwise 0.
     */
    public double getSlewRightY() {
        if (m_slewratelimiterRightY == null)
            return 0;
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
