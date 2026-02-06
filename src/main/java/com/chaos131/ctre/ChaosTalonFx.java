// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.ctre;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.chaos131.can.CanConstants.CanBusName;
import com.chaos131.can.CanConstants.CanId;
import com.chaos131.pid.PIDFValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** A TalonFX wrapper with automatic simulation support and helper functions. */
public class ChaosTalonFx extends TalonFX {
  private double m_gearRatio;
  private DCMotorSim m_motorSimModel;
  private boolean m_isMainSimMotor;
  private ChaosCanCoder m_attachedCanCoder;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);
  private final MotionMagicVoltage m_positionMotionMagicVoltage = new MotionMagicVoltage(0);
  private final DynamicMotionMagicVoltage m_positionDynamicMotionMagicVoltage =
      new DynamicMotionMagicVoltage(0, 0, 0);
  public final TalonFXConfiguration Configuration = new TalonFXConfiguration();

  /** Creates the new TalonFX wrapper WITHOUT simulation support. */
  public ChaosTalonFx(CanId canId, CanBusName canBus) {
    super(canId.id, canBus.name);
    this.m_gearRatio = 0.0;
    m_motorSimModel = null;
    m_isMainSimMotor = false;
  }

  /** Adds physical simulation support. */
  public void attachMotorSim(
      DCMotorSim dcMotorSim,
      double gearRatio,
      boolean isMainSimMotor,
      ChassisReference orientation,
      MotorType motorType) {
    this.m_gearRatio = gearRatio;
    m_motorSimModel = dcMotorSim;
    m_isMainSimMotor = isMainSimMotor;
    var talonFXSim = this.getSimState();
    talonFXSim.Orientation = orientation;
    talonFXSim.setMotorType(motorType);
  }

  /** Adds a CanCoder for syncing simulation values. */
  public void attachCanCoderSim(ChaosCanCoder canCoder) {
    m_attachedCanCoder = canCoder;
  }

  public void setSimAngle(Angle simAngle) {
    if (m_motorSimModel == null) {
      // Skip sim updates for motors without models
      return;
    }

    m_motorSimModel.setAngle(simAngle.in(Radians));
  }

  /**
   * Tells the motor to handle updating the sim state. Copied/inspired from:
   * https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/simulation/simulation-intro.html
   */
  public void simulationPeriodic() {
    if (m_motorSimModel == null) {
      // Skip sim updates for motors without models
      return;
    }

    TalonFXSimState talonFxSim = getSimState();

    // set the supply voltage of the TalonFX
    talonFxSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var motorVoltage = talonFxSim.getMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(motorVoltage);
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    if (m_isMainSimMotor) {
      if (m_attachedCanCoder != null) {
        CANcoderSimState canCoderSimState = m_attachedCanCoder.getSimState();
        canCoderSimState.setRawPosition(m_motorSimModel.getAngularPosition());
        canCoderSimState.setVelocity(m_motorSimModel.getAngularVelocity());
      }
    }

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFxSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(m_gearRatio));
    talonFxSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(m_gearRatio));
  }

  /** Applies/burns the configuration to the motor. */
  public void applyConfig() {
    getConfigurator().apply(Configuration);
  }

  /**
   * Tunes the PID default slot (0) PID of the motor.
   *
   * @deprecated this will be removed in favor of ChaosTalonFxTuner
   */
  public void tunePid(PIDFValue pidValue, double kg) {
    var slot0 = new Slot0Configs();
    slot0.kP = pidValue.P;
    slot0.kI = pidValue.I;
    slot0.kD = pidValue.D;
    slot0.kG = kg;
    Configuration.Slot0 = slot0;
    applyConfig();
  }

  /**
   * Tunes the PID and MotionMagic default slot (0).
   *
   * @deprecated this will be removed in favor of ChaosTalonFxTuner
   */
  public void tuneMotionMagic(PIDFValue pidValue, double kg, double ks, double kv, double ka) {
    var slot0 = new Slot0Configs();
    slot0.kP = pidValue.P;
    slot0.kI = pidValue.I;
    slot0.kD = pidValue.D;
    slot0.kG = kg;
    slot0.kS = ks;
    slot0.kV = kv;
    slot0.kA = ka;
    Configuration.Slot0 = slot0;
    applyConfig();
  }

  public void moveAtVelocity(AngularVelocity velocity) {
    m_velocityVoltage.Slot = 0;
    setControl(m_velocityVoltage.withVelocity(velocity));
  }

  /** Tells the motor controller to move to the target position. */
  public void moveToPosition(Angle position) {
    m_positionVoltage.Slot = 0;
    setControl(m_positionVoltage.withPosition(position));
  }

  /** Tells the motor controller to move to the target position. */
  public void moveToPosition(double position) {
    m_positionVoltage.Slot = 0;
    setControl(m_positionVoltage.withPosition(position));
  }

  /** Tells the motor controller to move to the target position. */
  public void moveToPosition(Angle position, int slot) {
    m_positionVoltage.Slot = slot;
    setControl(m_positionVoltage.withPosition(position));
  }

  /** Tells the motor controller to move to the target position using MotionMagic. */
  public void moveToPositionMotionMagic(Angle position) {
    m_positionMotionMagicVoltage.Slot = 0;
    setControl(m_positionMotionMagicVoltage.withPosition(position));
  }

  /** Tells the motor controller to move to the target position using MotionMagic. */
  public void moveToPositionMotionMagic(Angle position, Mass kg, Angle absoluteGravityAngle) {
    var currentRadians = absoluteGravityAngle.in(Radians);
    var currentKg = Math.cos(currentRadians) * kg.in(Kilograms);
    m_positionMotionMagicVoltage.Slot = 0;
    setControl(
        m_positionMotionMagicVoltage.withPosition(position).withFeedForward(Volts.of(currentKg)));
  }

  /**
   * Runs the motion magic curve for smoother transitions.
   *
   * @param position target position value in the consistent unit frame
   * @param velocity cruise velocity
   * @param acceleration max acceleration
   * @param jerk rate of change of the acceleration, you meanie
   */
  public void moveToPositionMotionMagic(
      double position, double velocity, double acceleration, double jerk) {
    m_positionDynamicMotionMagicVoltage.Slot = 0;
    m_positionDynamicMotionMagicVoltage.Velocity = velocity;
    m_positionDynamicMotionMagicVoltage.Acceleration = acceleration;
    m_positionDynamicMotionMagicVoltage.Jerk = jerk;
    setControl(m_positionDynamicMotionMagicVoltage.withPosition(position));
  }
}
