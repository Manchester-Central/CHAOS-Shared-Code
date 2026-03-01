// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.gamepads;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

/** A CHAOS Wrapper around `CommandXboxController` */
public class Gamepad extends CommandXboxController {
  private SlewRateLimiter m_slewratelimiterLeftX;
  private SlewRateLimiter m_slewratelimiterLeftY;
  private SlewRateLimiter m_slewratelimiterRightX;
  private SlewRateLimiter m_slewratelimiterRightY;
  private double m_deadbandRange;

  private Trigger m_leftXTrigger = createAxisTrigger(this::getLeftX);
  private Trigger m_leftYTrigger = createAxisTrigger(this::getLeftY);
  private Trigger m_rightXTrigger = createAxisTrigger(this::getRightX);
  private Trigger m_rightYTrigger = createAxisTrigger(this::getRightY);

  /**
   * @param port ID of the controller, typically 0 is driver, 1 is operator, etc
   * @param leftStickSlewRate number in the range [0,100]
   * @param rightStickSlewRate number in the range [0,100]
   */
  public Gamepad(int port, double leftStickSlewRate, double rightStickSlewRate) {
    super(port);
    m_slewratelimiterLeftX = new SlewRateLimiter(leftStickSlewRate);
    m_slewratelimiterLeftY = new SlewRateLimiter(leftStickSlewRate);
    m_slewratelimiterRightX = new SlewRateLimiter(rightStickSlewRate);
    m_slewratelimiterRightY = new SlewRateLimiter(rightStickSlewRate);
    m_deadbandRange = 0.05;
  }

  /**
   * @param port ID of the controller, typically 0 is driver, 1 is operator, etc
   */
  public Gamepad(int port) {
    this(port, 100.0, 100.0);
  }

  /**
   * @param value the value coming out of the controller
   * @return 0 if the value is in the deadband range
   */
  private double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, m_deadbandRange);
  }

  /**
   * @param value to set the deadband to, in the range of [0,1]
   */
  public void setDeadband(double value) {
    m_deadbandRange = value;
  }

  /**
   * @return Returns the deadbanded value of the joystick, and also attempts to update the slewrate
   *     calculator.
   */
  @Override
  public double getLeftX() {
    if (m_slewratelimiterLeftX != null) m_slewratelimiterLeftX.calculate(super.getLeftX());
    return applyDeadband(super.getLeftX());
  }

  /** Gets a trigger that activates if the joystick is moved (after deadband applied) */
  public Trigger leftX() {
    return m_leftXTrigger;
  }

  /**
   * @return Returns the deadbanded value of the joystick, and also attempts to update the slewrate
   *     calculator.
   */
  @Override
  public double getLeftY() {
    if (m_slewratelimiterLeftY != null) m_slewratelimiterLeftY.calculate(-super.getLeftY());
    return applyDeadband(-super.getLeftY());
  }

  /** Gets a trigger that activates if the joystick is moved (after deadband applied) */
  public Trigger leftY() {
    return m_leftYTrigger;
  }

  /**
   * @return Returns the deadbanded value of the joystick, and also attempts to update the slewrate
   *     calculator.
   */
  @Override
  public double getRightX() {
    if (m_slewratelimiterRightX != null) m_slewratelimiterRightX.calculate(super.getRightX());
    return applyDeadband(super.getRightX());
  }

  /** Gets a trigger that activates if the joystick is moved (after deadband applied) */
  public Trigger rightX() {
    return m_rightXTrigger;
  }

  /**
   * @return Returns the deadbanded value of the joystick, and also attempts to update the slewrate
   *     calculator.
   */
  @Override
  public double getRightY() {
    if (m_slewratelimiterRightY != null) m_slewratelimiterRightY.calculate(-super.getRightY());
    return applyDeadband(-super.getRightY());
  }

  /** Gets a trigger that activates if the joystick is moved (after deadband applied) */
  public Trigger rightY() {
    return m_rightYTrigger;
  }

  /**
   * @return Returns the slew rate derived value if there is a rate given, otherwise 0.
   */
  public double getSlewLeftX() {
    if (m_slewratelimiterLeftX == null) return 0;
    double newLeftX = m_slewratelimiterLeftX.calculate(super.getLeftX());
    return applyDeadband(newLeftX);
  }

  /**
   * @return Returns the slew rate derived value if there is a rate given, otherwise 0.
   */
  public double getSlewLeftY() {
    if (m_slewratelimiterLeftY == null) return 0;
    double newLeftY = m_slewratelimiterLeftY.calculate(-super.getLeftY());
    return applyDeadband(newLeftY);
  }

  /**
   * @return Returns the slew rate derived value if there is a rate given, otherwise 0.
   */
  public double getSlewRightX() {
    if (m_slewratelimiterRightX == null) return 0;
    double newSlewRightX = m_slewratelimiterRightX.calculate(super.getRightX());
    return applyDeadband(newSlewRightX);
  }

  /**
   * @return Returns the slew rate derived value if there is a rate given, otherwise 0.
   */
  public double getSlewRightY() {
    if (m_slewratelimiterRightY == null) return 0;
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

  /**
   * @return the angle of the left joystick, 0 degrees is straight right
   */
  public double getLeftAngle() {
    return Math.atan2(getLeftY(), getLeftX());
  }

  /**
   * @return the angle of the right joystick, 0 degrees is straight right
   */
  public double getRightAngle() {
    return Math.atan2(getRightY(), getRightX());
  }

  /**
   * @return the magnitude of the joystick tilt, using euclidean distance of the joystick values
   */
  public double getRightMagnitude() {
    return Math.hypot(getRightX(), getRightY());
  }

  /**
   * @return the magnitude of the joystick tilt, using euclidean distance of the joystick values
   */
  public double getLeftMagnitude() {
    return Math.hypot(getLeftX(), getLeftY());
  }

  /**
   * Creates a trigger from an exist - assumes a deadband is already applied to account for joystick
   * drift
   *
   * @param axis the axis value
   */
  public static Trigger createAxisTrigger(DoubleSupplier axis) {
    return new Trigger(() -> Math.abs(axis.getAsDouble()) > 0.0);
  }
}
