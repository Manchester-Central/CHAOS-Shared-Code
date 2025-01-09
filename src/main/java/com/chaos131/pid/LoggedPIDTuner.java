package com.chaos131.pid;

import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * A Logged Dashboard number specific for PID Values. This is useful for tracking PID values during
 * testing, but should be replaced with a basic PID structure once settled. Extends the PIDTuner to
 * make it easier to swap between the two.
 */
public class LoggedPIDTuner extends PIDTuner {
  /** The P, I, D, and F values broken into their own logged values */
  private LoggedDashboardNumber m_p, m_i, m_d, m_f;

  /**
   * Creates a PID Tuner with AdvantageKit Logging built in
   *
   * @param key name of the pid controller
   * @param _p value for the controller
   * @param _i value for the controller
   * @param _d value for the controller
   * @param pidfUpdater callback function to update a device when the values change
   */
  public LoggedPIDTuner(
      String key, double _p, double _i, double _d, Consumer<PIDFValue> pidfUpdater) {
    this(key, _p, _i, _d, 0, pidfUpdater);
  }

  /**
   * Creates a PID Tuner with AdvantageKit Logging built in
   *
   * @param key name of the pid controller
   * @param _p value for the controller
   * @param _i value for the controller
   * @param _d value for the controller
   * @param _f value for the controller
   * @param pidfUpdater callback function to update a device when the values change
   */
  public LoggedPIDTuner(
      String key, double _p, double _i, double _d, double _f, Consumer<PIDFValue> pidfUpdater) {
    super(key, true, _p, _i, _d, _f, pidfUpdater);
    m_p = new LoggedDashboardNumber(key + "_p", _p);
    m_f = new LoggedDashboardNumber(key + "_f", _f);
    m_i = new LoggedDashboardNumber(key + "_i", _i);
    m_d = new LoggedDashboardNumber(key + "_d", _d);
  }

  @Override
  protected void setDefaults(double defaultP, double defaultI, double defaultD, double defaultF) {}

  /** Collects the values from each of the LoggedDashboardNumbers */
  @Override
  public PIDFValue toPIDFValue() {
    return new PIDFValue(m_p.get(), m_i.get(), m_d.get(), m_f.get());
  }
}
