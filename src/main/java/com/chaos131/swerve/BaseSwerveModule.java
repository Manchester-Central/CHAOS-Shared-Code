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

/**
 * An individual swerve module that composes the swerve drive system. There are typically 4 modules on a robot.
 */
public abstract class BaseSwerveModule {
    /**
     * Name of the swerve module, typically this is FrontLeft, FrontRight, etc
     */
    private String m_name;
    /**
     * Offset of rotation axis (+z direction) of the wheel in the module
     */
    private Translation2d m_translation;
    /**
     * Angle the wheels move towards to lock the robot in place
     * 
     * <p> Each wheel is pointed away from the center of the robot assuming standard coordinate system
     * <p> +x is forward on the robot
     * <p> +y is left on the robot
     */
    private Rotation2d m_xModeAngle;

    /**
     * Tracks the distance the robot has moved for the purposes of the simulation
     */
    private double m_simdistance;
    /**
     * Tracks the desired state of the individual module
     */
    protected SwerveModuleState m_targetState;
    /**
     * The tick rate of the system for simulation purposes, should match the actual update speed of a real robot
     */
    private double m_updatePeriodMs = 50;

    /**
     * Filter used to denoise the angle position
     */
    private LinearFilter m_absoluteAngleDegreesRollingAverage = LinearFilter.movingAverage(100);
    /**
     * Resulting values from the Angle Filter
     */
    protected double m_absoluteAngleDegreesRollingAverageValue = 0;

    /**
     * Creates a new SwerveModule
     * 
     * @param name - Name of the module used for Driver Station and Dashboard
     * @param translation - Cartesian position relative to robot origin
     */
    public BaseSwerveModule(String name, Translation2d translation) {
        m_name = name;
        m_translation = translation;
        m_simdistance = 0;
        m_targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        m_xModeAngle = Rotation2d.fromRadians(Math.atan2(translation.getY(), translation.getX()));
    }

    /**
     * @param coachTab I don't know what this is
     */
    public abstract void addCoachTabDashboardValues(ShuffleboardTab coachTab);
    /**
     * Sets up the module for driving
     */
    public abstract void driverModeInit();
    /**
     * Sets up the module for driving to specific locations.
     */
    public abstract void driveToPositionInit();
    /**
     * @param update the values to set the velocity controller to
     */
    public abstract void updateVelocityPIDConstants(PIDFValue update);
    /**
     * @param update the values to set the angle controller to
     */
    public abstract void updateAnglePIDConstants(PIDFValue update);
    /**
     * @param percentSpeed the value from [0,1] to set the velocity controller to
     */
    public abstract void setPercentSpeed(double percentSpeed);

    /**
     * @return the detected distance travelled
     */
    protected abstract double getEncoderDistance_m();
    /**
     * @return the current velocity for this module
     */
    protected abstract double getEncoderVelocity_mps();
    /**
     * @param velocity_mps the desired velocity for this module
     */
    protected abstract void setTargetVelocity_mps(double velocity_mps);
    /**
     * @return naively converted angle from the encoder position
     */
    protected abstract Rotation2d getEncoderAngle();
    /**
     * @param angle value describing the angle of this module
     */
    protected abstract void setTargetAngle(Rotation2d angle);
    /**
     * @return practical rotation of the module
     */
    protected abstract Rotation2d getAbsoluteAngle();
    /**
     * Stops the motor controlling the swerve angle
     */
    protected abstract void stopAngleMotor();
    /**
     * Stops the motor controlling the driving speed / translation
     */
    protected abstract void stopSpeedMotor();
    /**
     * Sends data to the Dashboard
     */
    protected abstract void updateDashboard();
    /**
     * @param absoluteAngle recalibrates the rotation given a measured angle
     */
    protected abstract void recalibrateWithFilteredAbsoluteAngle(Rotation2d absoluteAngle);

    /**
     * @return returns the name of the Swerve Module
     */
    public String getName() {
        return m_name;
    }

    /**
     * @param field name of the swerve module property
     * @return Returns the name of the Driver Station key for the given field
     */
    protected String getDSKey(String field) {
        return "Swerve Module " + m_name + "/" + field;
    }

    /**
     * @param updatePeriodMs value to change the simulation update speed by
     */
    public void setSimUpdatePeriod(double updatePeriodMs) {
        m_updatePeriodMs = updatePeriodMs;
    }

    /**
     * @param state the state to set this swerve module's rotation angles and velocity values
     */
    public void setTarget(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getModuleState().angle);
        var targetAngle = Rotation2d.fromDegrees(closestTarget(getModuleState().angle.getDegrees(), state.angle.getDegrees()));

        setTargetAngle(targetAngle);
        setTargetVelocity_mps(state.speedMetersPerSecond);
        m_targetState = state;
    }

    /**
     * Puts the robot into X mode to lock in place
     */
    public void setXMode() {
        setTarget(new SwerveModuleState(0, m_xModeAngle));
    }

    /**
     * Stops all motors and resets the target state
     */
    public void stop() {
        stopAngleMotor();
        stopSpeedMotor();
        m_targetState = new SwerveModuleState(0, getModuleState().angle); 
    }

    /**
     * @return Returns the current velocity and angle on a live robot, otherwise the target state while in simulation
     */
    public SwerveModuleState getModuleState() {
        if (RobotBase.isSimulation()) {
            return m_targetState;
        }
        return new SwerveModuleState(getEncoderVelocity_mps(), getEncoderAngle());
    }

    /**
     * @return the wheel offsets for the module relative to the robot origin
     */
    public Translation2d getTranslation() {
        return m_translation;
    }

    /**
     * @return the field position of the swerve module
     */
    public SwerveModulePosition getPosition() {
        if (RobotBase.isSimulation()) {
            m_simdistance = m_simdistance + m_targetState.speedMetersPerSecond / m_updatePeriodMs;
            return new SwerveModulePosition(m_simdistance, m_targetState.angle);
        }
        return new SwerveModulePosition(getEncoderDistance_m(), getEncoderAngle());
    }

    /**
     * Since this is not a WPILib Subsystem, this does not get triggered periodically on its own.
     * An owning thread or data structure must call this periodic function.
     */
    public void periodic() {
        m_absoluteAngleDegreesRollingAverageValue = m_absoluteAngleDegreesRollingAverage.calculate(getAbsoluteAngle().getDegrees());
        updateDashboard();
    }

    /**
     * Recalibrates the swerve module
     */
    public void recalibrate() {
        recalibrateWithFilteredAbsoluteAngle(Rotation2d.fromDegrees(m_absoluteAngleDegreesRollingAverageValue));
    }

    /**
     * This function takes in the current angle read by the encoder and a target angle for the robot to move to.
     * The target angle will be between -180 and 180, but this function will scale it up so it is an equivalent
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
//"If you don’t agree to focus, you’re going to the business team." -Josh 2/21/2023
