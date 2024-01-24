// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.swerve.implementation;

import com.chaos131.pid.PIDFValue;
import com.chaos131.swerve.BaseSwerveModule;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Creates a Swerve Module using TalonFxs and a CANcoder (CHAOS's 2022 and 2024 bots used this configuration) */
public class TalonFxAndCancoderSwerveModule extends BaseSwerveModule {
    private TalonFXConfiguration m_velocityConfig = new TalonFXConfiguration();
    private TalonFXConfiguration m_angleConfig = new TalonFXConfiguration();
    
    private VelocityVoltage m_velocityVoltageMps = new VelocityVoltage(0);
    private PositionVoltage m_positionVoltageRotations = new PositionVoltage(0);

    public TalonFxAndCancoderSwerveModule(String name, Translation2d translation, Rotation2d xModeAngle) {
        super(name, translation, xModeAngle);
    }

    @Override
    public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addCoachTabDashboardValues'");
    }

    @Override
    public void driverModeInit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driverModeInit'");
    }

    @Override
    public void driveToPositionInit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveToPositionInit'");
    }

    public void UpdateVelocityPIDConstants(PIDFValue update) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'UpdateVelocityPIDConstants'");
    }

    public void UpdateAnglePIDConstants(PIDFValue update) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'UpdateAnglePIDConstants'");
    }

    @Override
    protected double getEncoderDistance_m() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEncoderDistance_m'");
    }

    @Override
    protected double getEncoderVelocity_mps() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEncoderVelocity_mps'");
    }

    @Override
    protected void setTargetVelocity_mps(double velocity_mps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetVelocity_mps'");
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEncoderAngle'");
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetAngle'");
    }

    @Override
    protected Rotation2d getAbsoluteAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAbsoluteAngle'");
    }

    @Override
    protected void stopAngleMotor() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopAngleMotor'");
    }

    @Override
    protected void stopVelocityMotor() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopVelocityMotor'");
    }

    @Override
    protected void updateDashboard() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateDashboard'");
    }

    @Override
    protected void recalibrateWithFilteredAbsoluteAngle(Rotation2d absoluteAngle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'recalibrateWithFilteredAbsoluteAngle'");
    }

	@Override
	public void updateVelocityPIDConstants(PIDFValue update) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'updateVelocityPIDConstants'");
	}

	@Override
	public void updateAnglePIDConstants(PIDFValue update) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'updateAnglePIDConstants'");
	}

}
