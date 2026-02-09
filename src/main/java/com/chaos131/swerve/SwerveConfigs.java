package com.chaos131.swerve;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Configuration container that holds debug states, PID values, tolerance values, and any other
 * Swerve Tuning values.
 *
 * @deprecated CHAOS has moved onto using <a
 *     href="https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/">AdvantageKit's
 *     TalonFx template</a>
 */
@Deprecated(since = "2026.1", forRemoval = true)
public class SwerveConfigs {
  /** These configs should be created and configured before creating the swerve drive class */
  public SwerveConfigs() {}

  /** Debug mode */
  private boolean m_isDebugMode = false;

  public boolean IsDebugMode() {
    return m_isDebugMode;
  }

  public SwerveConfigs setDebugMode(boolean isDebugMode) {
    m_isDebugMode = isDebugMode;
    return this;
  }

  /** Default translation PID values */
  private PIDValue m_defaultTranslationPIDValues = new PIDValue(0.6, 0.05, 0.1);

  public PIDValue defaultTranslationPIDValues() {
    return m_defaultTranslationPIDValues;
  }

  public SwerveConfigs setDefaultTranslationPIDValues(PIDValue defaultTranslationPIDValues) {
    m_defaultTranslationPIDValues = defaultTranslationPIDValues;
    return this;
  }

  /** Default DriveToTargetTolerance */
  private double m_defaultDriveToTargetTolerance = 0.03;

  public double defaultDriveToTargetTolerance() {
    return m_defaultDriveToTargetTolerance;
  }

  public SwerveConfigs setDefaultDriveToTargetTolerance(double defaultDriveToTargetTolerance) {
    m_defaultDriveToTargetTolerance = defaultDriveToTargetTolerance;
    return this;
  }

  /** Default rotation PID values */
  private PIDValue m_defaultRotationPIDValues = new PIDValue(0.01, 0.0001, 0.00);

  public PIDValue defaultRotationPIDValues() {
    return m_defaultRotationPIDValues;
  }

  public SwerveConfigs setDefaultRotationPIDValues(PIDValue defaultRotationPIDValues) {
    m_defaultRotationPIDValues = defaultRotationPIDValues;
    return this;
  }

  /** Default DriveToTargetTolerance */
  private Rotation2d m_defaultRotationTolerance = Rotation2d.fromDegrees(3.0);

  public Rotation2d defaultRotationTolerance() {
    return m_defaultRotationTolerance;
  }

  public SwerveConfigs setDefaultRotationTolerance(Rotation2d defaultRotationTolerance) {
    m_defaultRotationTolerance = defaultRotationTolerance;
    return this;
  }

  /** MaxRobotSpeed_mps */
  private LinearVelocity m_maxRobotSpeed = Units.MetersPerSecond.of(4.0);

  public LinearVelocity maxRobotSpeed() {
    return m_maxRobotSpeed;
  }

  public SwerveConfigs setMaxRobotSpeed(LinearVelocity maxRobotSpeed) {
    m_maxRobotSpeed = maxRobotSpeed;
    return this;
  }

  /** MaxRobotRotation */
  private AngularVelocity m_maxRobotRotation = Units.RadiansPerSecond.of(3.0);

  public AngularVelocity maxRobotRotation() {
    return m_maxRobotRotation;
  }

  public SwerveConfigs setMaxRobotRotation(AngularVelocity maxRobotRotation) {
    m_maxRobotRotation = maxRobotRotation;
    return this;
  }

  /** UpdateFrequency_Hz */
  private double m_updateFrequency_hz = 50;

  public double updateFrequency_hz() {
    return m_updateFrequency_hz;
  }

  public SwerveConfigs setUpdateFrequency_hz(double updateFrequency_hz) {
    m_updateFrequency_hz = updateFrequency_hz;
    return this;
  }

  /** defaultModuleVelocityPIDFValues */
  private PIDFValue m_defaultModuleVelocityPIDFValues = new PIDFValue(0.0375, 0.0, 0.0, 0.054);

  public PIDFValue defaultModuleVelocityPIDFValues() {
    return m_defaultModuleVelocityPIDFValues;
  }

  public SwerveConfigs setDefaultModuleVelocityPIDFValues(
      PIDFValue defaultModuleVelocityPIDFValues) {
    m_defaultModuleVelocityPIDFValues = defaultModuleVelocityPIDFValues;
    return this;
  }

  /** ModuleAngle_PID_Tuner */
  private PIDValue m_defaultModuleAnglePIDValues = new PIDValue(0.01, 0.0001, 0.00);

  /**
   * @return the angle pid structure
   */
  public PIDValue defaultModuleAnglePIDValues() {
    return m_defaultModuleAnglePIDValues;
  }

  /**
   * @param defaultModuleAnglePIDValues pid structure
   * @return this object for member chaining
   */
  public SwerveConfigs setDefaultModuleAnglePIDValues(PIDValue defaultModuleAnglePIDValues) {
    m_defaultModuleAnglePIDValues = defaultModuleAnglePIDValues;
    return this;
  }

  /** Default alliance */
  private Alliance m_defaultAlliance = Alliance.Blue;

  /**
   * @return default alliance
   */
  public Alliance defaultAlliance() {
    return m_defaultAlliance;
  }

  /**
   * Sets the default alliance for the coordinate system (which color is closest to 0,0)
   *
   * @param alliance being set
   * @return this object for member chaining
   */
  public SwerveConfigs setDefaultAlliance(Alliance alliance) {
    m_defaultAlliance = alliance;
    return this;
  }
}
