package com.chaos131.swerve;

import com.chaos131.pid.PIDFValue;
import com.chaos131.pid.PIDValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SwerveConfigs {
    
    /**
     * These configs should be created and configured before creating the swerve drive class
     */
    public SwerveConfigs() {

    }

    // Debug mode
    private boolean m_isDebugMode = false;
    public boolean IsDebugMode() {
        return m_isDebugMode;
    }
    public SwerveConfigs setDebugMode(boolean isDebugMode) {
        m_isDebugMode = isDebugMode;
        return this;
    }

    // Default translation PID values
    private PIDValue m_defaultTranslationPIDValues = new PIDValue(0.6, 0.05, 0.1);
    public PIDValue defaultTranslationPIDValues() {
        return m_defaultTranslationPIDValues;
    }
    public SwerveConfigs setDefaultTranslationPIDValues(PIDValue defaultTranslationPIDValues) {
        m_defaultTranslationPIDValues = defaultTranslationPIDValues;
        return this;
    }

    // Default DriveToTargetTolerance
    private double m_defaultDriveToTargetTolerance = 0.03;
    public double defaultDriveToTargetTolerance() {
        return m_defaultDriveToTargetTolerance;
    }
    public SwerveConfigs setDefaultDriveToTargetTolerance(double defaultDriveToTargetTolerance) {
        m_defaultDriveToTargetTolerance = defaultDriveToTargetTolerance;
        return this;
    }

    // Default rotation PID values
    private PIDValue m_defaultRotationPIDValues = new PIDValue(0.01, 0.0001, 0.00);
    public PIDValue defaultRotationPIDValues() {
        return m_defaultRotationPIDValues;
    }
    public SwerveConfigs setDefaultRotationPIDValues(PIDValue defaultRotationPIDValues) {
        m_defaultRotationPIDValues = defaultRotationPIDValues;
        return this;
    }

    // Default DriveToTargetTolerance
    private Rotation2d m_defaultRotationTolerance = Rotation2d.fromDegrees(3.0);
    public Rotation2d defaultRotationTolerance() {
        return m_defaultRotationTolerance;
    }
    public SwerveConfigs setDefaultRotationTolerance(Rotation2d defaultRotationTolerance) {
        m_defaultRotationTolerance = defaultRotationTolerance;
        return this;
    }

    // MaxRobotSpeed_mps
    private double m_maxRobotSpeed_mps = 4.0;
    public double maxRobotSpeed_mps() {
        return m_maxRobotSpeed_mps;
    }
    public SwerveConfigs setMaxRobotSpeed_mps(double maxRobotSpeed_mps) {
        m_maxRobotSpeed_mps = maxRobotSpeed_mps;
        return this;
    }

    // MaxRobotRotation_radps
    private double m_maxRobotRotation_radps = 3;
    public double maxRobotRotation_radps() {
        return m_maxRobotRotation_radps;
    }
    public SwerveConfigs setMaxRobotRotation_radps(double maxRobotRotation_radps) {
        m_maxRobotRotation_radps = maxRobotRotation_radps;
        return this;
    }

    // UpdateFrequency_Hz
    private double m_updateFrequency_hz = 50;
    public double updateFrequency_hz() {
        return m_updateFrequency_hz;
    }
    public SwerveConfigs setUpdateFrequency_hz(double updateFrequency_hz) {
        m_updateFrequency_hz = updateFrequency_hz;
        return this;
    }

    // defaultModuleVelocityPIDFValues
    private PIDFValue m_defaultModuleVelocityPIDFValues = new PIDFValue(0.0375, 0.0, 0.0, 0.054);
    public PIDFValue defaultModuleVelocityPIDFValues() {
        return m_defaultModuleVelocityPIDFValues;
    }
    public SwerveConfigs setDefaultModuleVelocityPIDFValues(PIDFValue defaultModuleVelocityPIDFValues) {
        m_defaultModuleVelocityPIDFValues = defaultModuleVelocityPIDFValues;
        return this;
    }

    // ModuleAngle_PID_Tuner
    private PIDValue m_defaultModuleAnglePIDValues = new PIDValue(0.01, 0.0001, 0.00);
    public PIDValue defaultModuleAnglePIDValues() {
        return m_defaultModuleAnglePIDValues;
    }
    public SwerveConfigs setDefaultModuleAnglePIDValues(PIDValue defaultModuleAnglePIDValues) {
        m_defaultModuleAnglePIDValues = defaultModuleAnglePIDValues;
        return this;
    }

    // Default color
    private Alliance m_defaultAlliance = Alliance.Blue;
    public Alliance defaultAlliance() {
        return m_defaultAlliance;
    }
    /** Sets the default alliance for the coordinate system (which color is closest to 0,0) */
    public SwerveConfigs setDefaultAlliance(Alliance alliance) {
        m_defaultAlliance = alliance;
        return this;
    }
}
