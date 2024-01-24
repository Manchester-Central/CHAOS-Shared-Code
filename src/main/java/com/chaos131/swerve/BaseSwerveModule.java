// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.swerve;

import com.chaos131.pid.PIDFValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class BaseSwerveModule {
    private String m_name;
    private Translation2d m_translation;
    private Rotation2d m_xModeAngle; // FUTURE: This can be calculated from the translation

    private double m_simdistance;
    protected SwerveModuleState m_targetState;
    private double m_updateFrequency_hz = 50;

    private LinearFilter m_absoluteAngleDegreesRollingAverage = LinearFilter.movingAverage(100);
    protected double m_absoluteAngleDegreesRollingAverageValue = 0;

    /** Creates a new SwerveModule. */
    public BaseSwerveModule(String name, Translation2d translation, Rotation2d xModeAngle) {
        m_name = name;
        m_translation = translation;
        m_xModeAngle = xModeAngle;
        m_simdistance = 0;
        m_targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    public abstract void addCoachTabDashboardValues(ShuffleboardTab coachTab);

    public abstract void driverModeInit();

    public abstract void driveToPositionInit();

    public abstract void updateVelocityPIDConstants(PIDFValue update);

    public abstract void updateAnglePIDConstants(PIDFValue update);

    protected abstract double getEncoderDistance_m();

    protected abstract double getEncoderVelocity_mps();

    protected abstract void setTargetVelocity_mps(double velocity_mps);

    protected abstract Rotation2d getEncoderAngle();

    protected abstract void setTargetAngle(Rotation2d angle);

    protected abstract Rotation2d getAbsoluteAngle();

    protected abstract void stopAngleMotor();

    protected abstract void stopVelocityMotor();

    protected abstract void updateDashboard();

    protected abstract void recalibrateWithFilteredAbsoluteAngle(Rotation2d absoluteAngle);

    public String getName() {
        return m_name;
    }

    protected String getDSKey(String field) {
        return "Swerve Module " + m_name + "/" + field;
    }

    public void setSimUpdateFrequency_hz(double updateFrequency_hz) {
        m_updateFrequency_hz = updateFrequency_hz;
    }

    public void setTarget(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getModuleState().angle);
        var targetAngle =
                Rotation2d.fromDegrees(closestTarget(getModuleState().angle.getDegrees(), state.angle.getDegrees()));

        setTargetAngle(targetAngle);
        setTargetVelocity_mps(state.speedMetersPerSecond);
        m_targetState = state;
    }

    public void setXMode() {
        setTarget(new SwerveModuleState(0, m_xModeAngle));
    }

    public void stop() {
        stopAngleMotor();
        stopVelocityMotor();
        m_targetState = new SwerveModuleState(0, getModuleState().angle);
    }

    public SwerveModuleState getModuleState() {
        if (RobotBase.isSimulation()) {
            return m_targetState;
        }
        return new SwerveModuleState(getEncoderVelocity_mps(), getEncoderAngle());
    }

    public Translation2d getTranslation() {
        return m_translation;
    }

    public SwerveModulePosition getPosition() {
        if (RobotBase.isSimulation()) {
            m_simdistance = m_simdistance + m_targetState.speedMetersPerSecond / m_updateFrequency_hz;
            return new SwerveModulePosition(m_simdistance, m_targetState.angle);
        }
        return new SwerveModulePosition(getEncoderDistance_m(), getEncoderAngle());
    }

    public void periodic() {
        m_absoluteAngleDegreesRollingAverageValue = m_absoluteAngleDegreesRollingAverage.calculate(
                getAbsoluteAngle().getDegrees());
        updateDashboard();
    }

    public void recalibrate() {
        recalibrateWithFilteredAbsoluteAngle(Rotation2d.fromDegrees(m_absoluteAngleDegreesRollingAverageValue));
    }

    /**
     * This function takes in the current angle read by the encoder and a target angle for the robot to move to.
     * The target angle will be between -PI and PI, but this function will scale it up so it is an equivalent
     * angle that is closer to the current encoder angle. It will return this "optimized" angle to avoid
     * the wheels overspinning.
     * @param currentModuleAngle_deg - The current swerve module's angle in degrees
     * @param targetAngle_deg - The target angle in degrees
     * @return The swerve module target angle
     */
    public static double closestTarget(double currentModuleAngle_deg, double targetAngle_deg) {
        Rotation2d currentModuleAngle = Rotation2d.fromDegrees(currentModuleAngle_deg);
        Rotation2d targetAngle = Rotation2d.fromDegrees(targetAngle_deg);
        Rotation2d angleDifference = currentModuleAngle.minus(targetAngle);
        return currentModuleAngle_deg - angleDifference.getDegrees();
    }
}
// "If you don’t agree to focus, you’re going to the business team." -Josh 2/21/2023
