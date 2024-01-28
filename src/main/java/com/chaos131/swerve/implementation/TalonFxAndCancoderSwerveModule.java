// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.swerve.implementation;

import com.chaos131.pid.PIDFValue;
import com.chaos131.swerve.BaseSwerveModule;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** Creates a Swerve Module using TalonFxs and a CANcoder (CHAOS's 2022 and 2024 bots used this configuration) */
public class TalonFxAndCancoderSwerveModule extends BaseSwerveModule {

    public static class SpeedControllerConfig {
        public final int canId;
        public final InvertedValue motorDirection;
        public final double speedGearRatio;
        public final double wheelCircumference;

        public SpeedControllerConfig(
            int canId,
            InvertedValue motorDirection,
            double speedGearRatio,
            double wheelCircumference
        ) {
            this.canId = canId;
            this.motorDirection = motorDirection;
            this.speedGearRatio = speedGearRatio;
            this.wheelCircumference = wheelCircumference;
        }
    }

    public static class AngleControllerConfig {
        public final int canId;
        public final InvertedValue motorDirection;
        public final double angleGearRatio;

        public AngleControllerConfig(
            int canId,
            InvertedValue motorDirection,
            double angleGearRatio
        ) {
            this.canId = canId;
            this.motorDirection = motorDirection;
            this.angleGearRatio = angleGearRatio;
        }
    }

    public static class AbsoluteEncoderConfig {
        public final int canId;
        public final SensorDirectionValue sensorDirection;
        public final Rotation2d absoluteAngleOffset;

        public AbsoluteEncoderConfig(
            int canId,
            SensorDirectionValue sensorDirection,
            Rotation2d absoluteAngleOffset
        ) {
            this.canId = canId;
            this.sensorDirection = sensorDirection;
            this.absoluteAngleOffset = absoluteAngleOffset;
        }
    }

    public static class DriveConfig {
        public final double driverModeClosedLoopRampRatePeriod;
        public final double driveToPositionClosedLoopRampRatePeriod;
        public final Rotation2d xModeAngle;

        public DriveConfig(
            double driverModeClosedLoopRampRatePeriod,
            double driveToPositionClosedLoopRampRatePeriod,
            Rotation2d xModeAngle
        ) {
            this.driverModeClosedLoopRampRatePeriod = driverModeClosedLoopRampRatePeriod;
            this.driveToPositionClosedLoopRampRatePeriod = driveToPositionClosedLoopRampRatePeriod;
            this.xModeAngle = xModeAngle;
        }
    }
    
    protected CANcoder m_absoluteEncoder;
    protected TalonFX m_speedController;
    protected TalonFX m_angleController;

    protected TalonFXConfiguration m_speedConfig = new TalonFXConfiguration();
    protected TalonFXConfiguration m_angleConfig = new TalonFXConfiguration();
    protected CANcoderConfiguration m_cancoderConfig = new CANcoderConfiguration();
    
    protected VelocityVoltage m_velocityVoltageMps = new VelocityVoltage(0);
    protected PositionVoltage m_positionVoltageRotations = new PositionVoltage(0);

    protected DriveConfig m_driveConfig;

    /**
     * Creates a new swerve module
     * @param name the name of the module "FL", "FrontLeft" or whatever standard 
     * @param translation the position of the module compared to the center of the robot
     * @param speedControllerConfig the config for the speed controller
     * @param angleControllerConfig the config for the angle controller
     * @param absoluteEncoderConfig the config for the absolute encoder
     * @param driveConfig the options for driving the robot
     */
    public TalonFxAndCancoderSwerveModule(
        String name,
        Translation2d translation,
        SpeedControllerConfig speedControllerConfig,
        AngleControllerConfig angleControllerConfig,
        AbsoluteEncoderConfig absoluteEncoderConfig,
        DriveConfig driveConfig
    ) {
        super(name, translation, driveConfig.xModeAngle);

        // initialize member variables
        m_speedController = new TalonFX(speedControllerConfig.canId);
        m_angleController = new TalonFX(angleControllerConfig.canId);
        m_absoluteEncoder = new CANcoder(absoluteEncoderConfig.canId);
        m_driveConfig = driveConfig;

        // CANCoder config
        m_cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderConfig.absoluteAngleOffset.getRotations();
        m_cancoderConfig.MagnetSensor.SensorDirection = absoluteEncoderConfig.sensorDirection;
        m_absoluteEncoder.getConfigurator().apply(m_cancoderConfig);
        m_absoluteEncoder.getAbsolutePosition().setUpdateFrequency(4);

        // speed controller config
        m_speedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_speedConfig.MotorOutput.Inverted = speedControllerConfig.motorDirection;
        m_speedConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_speedConfig.Feedback.SensorToMechanismRatio = speedControllerConfig.speedGearRatio / speedControllerConfig.wheelCircumference;
        m_speedController.getConfigurator().apply(m_speedConfig);

        // angle controller config
        m_angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_angleConfig.MotorOutput.Inverted = angleControllerConfig.motorDirection;
        m_angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_angleConfig.Feedback.SensorToMechanismRatio = angleControllerConfig.angleGearRatio;
        m_angleController.getConfigurator().apply(m_angleConfig);

        // set intitial angle value
        m_angleController.setPosition(getAbsoluteAngle().getRotations());
    }

    @Override
    protected double getEncoderDistance_m() {
        return m_speedController.getPosition().getValueAsDouble();
    }

    @Override
    protected double getEncoderVelocity_mps() {
        return m_speedController.getVelocity().getValueAsDouble();
    }

    @Override
    protected void setTargetVelocity_mps(double velocity_mps) {
        m_velocityVoltageMps.Slot = 0;
        m_speedController.setControl(m_velocityVoltageMps.withVelocity(velocity_mps));
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromRotations(m_angleController.getPosition().getValueAsDouble());
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        m_positionVoltageRotations.Slot = 0;
        m_angleController.setControl(m_positionVoltageRotations.withPosition(angle.getRotations()));
    }

    @Override
    protected Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromRotations(m_absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    protected void stopAngleMotor() {
        m_angleController.stopMotor();
    }

    @Override
    protected void stopSpeedMotor() {
        m_speedController.stopMotor();
    }

    @Override
    protected void recalibrateWithFilteredAbsoluteAngle(Rotation2d absoluteAngle) {
        m_angleController.setPosition(absoluteAngle.getRotations());
    }

	@Override
	public void updateVelocityPIDConstants(PIDFValue update) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = update.P;
        slot0Configs.kI = update.I;
        slot0Configs.kD = update.D;
        slot0Configs.kV = update.F;
        slot0Configs.kS = 0.05;

        m_speedController.getConfigurator().apply(slot0Configs);
	}

	@Override
	public void updateAnglePIDConstants(PIDFValue update) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = update.P;
        slot0Configs.kI = update.I;
        slot0Configs.kD = update.D;
        slot0Configs.kV = update.F;

        m_angleController.getConfigurator().apply(slot0Configs);
	}

    @Override
    public void driverModeInit() {
        m_speedConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = m_driveConfig.driverModeClosedLoopRampRatePeriod;
        m_speedController.getConfigurator().apply(m_speedConfig);
    }

    @Override
    public void driveToPositionInit() {
        m_speedConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = m_driveConfig.driveToPositionClosedLoopRampRatePeriod;
        m_speedController.getConfigurator().apply(m_speedConfig);
    }

    @Override
    public void addCoachTabDashboardValues(ShuffleboardTab coachTab) {}

    @Override
    protected void updateDashboard() {}

}
