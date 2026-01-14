// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.util;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.chaos131.pid.PIDFValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** A TalonFX wrapper with automatic simulation support and helper functions. */
public class ChaosTalonFxs extends TalonFXS {
  private double m_gearRatio;
  private DCMotorSim m_motorSimModel;
  private ChassisReference m_simDirection;
  private boolean m_isMainSimMotor;
  private ChaosCanCoder m_attachedCanCoder;
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0);
  private final MotionMagicVoltage m_positionMotionMagicVoltage = new MotionMagicVoltage(0);
  private final DynamicMotionMagicVoltage m_positionDynamicMotionMagicVoltage =
      new DynamicMotionMagicVoltage(0, 0, 0);
  public final TalonFXSConfiguration Configuration = new TalonFXSConfiguration();
  private double m_lastUserSetSpeed = 0.0;

  /** Creates the new TalonFX wrapper WITHOUT simulation support. */
  public ChaosTalonFxs(int canId, String canBus) {
    super(canId, canBus);
    this.m_gearRatio = 0.0;
    m_motorSimModel = null;
    m_isMainSimMotor = false;
  }

  /** Adds physical simulation support. */
  public void attachMotorSim(
      DCMotorSim dcMotorSim,
      double gearRatio,
      ChassisReference simDirection,
      boolean isMainSimMotor) {
    this.m_gearRatio = gearRatio;
    m_motorSimModel = dcMotorSim;
    m_simDirection = simDirection;
    m_isMainSimMotor = isMainSimMotor;
  }

  /** Adds a CanCoder for syncing simulation values. */
  public void attachCanCoderSim(ChaosCanCoder canCoder) {
    m_attachedCanCoder = canCoder;
  }

  public void setSpeed(double speed) {
    m_lastUserSetSpeed = speed;
    super.set(speed);
  }

  public double getSpeed() {
    // if (Robot.isSimulation()) {
    //   return m_lastUserSetSpeed;
    // }
    return super.get();
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
  public void simUpdate() {
    if (m_motorSimModel == null) {
      // Skip sim updates for motors without models
      return;
    }

    var talonFxSim = getSimState();
    // TODO: talonFxSim.Orientation = m_simDirection;

    // set the supply voltage of the TalonFX
    talonFxSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    if (m_isMainSimMotor) {
      // get the motor voltage of the TalonFX
      var motorVoltage = talonFxSim.getMotorVoltage();

      // use the motor voltage to calculate new position and velocity
      // using WPILib's DCMotorSim class for physics simulation
      m_motorSimModel.setInputVoltage(motorVoltage);
      m_motorSimModel.update(0.020); // assume 20 ms loop time

      if (m_attachedCanCoder != null) {
        var canCoderSimState = m_attachedCanCoder.getSimState();
        canCoderSimState.setRawPosition(m_motorSimModel.getAngularPositionRotations());
        canCoderSimState.setVelocity(
            Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec()));
      }
    }

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFxSim.setRawRotorPosition(m_gearRatio * m_motorSimModel.getAngularPositionRotations());
    talonFxSim.setRotorVelocity(
        m_gearRatio * Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec()));
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

  /** Tells the motor controller to move to the target position. */
  public void moveToPosition(double position) {
    m_positionVoltage.Slot = 0;
    setControl(m_positionVoltage.withPosition(position));
  }

  /** Tells the motor controller to move to the target position. */
  public void moveToPosition(double position, int slot) {
    m_positionVoltage.Slot = slot;
    setControl(m_positionVoltage.withPosition(position));
  }

  /** Tells the motor controller to move to the target position using MotionMagic. */
  public void moveToPositionMotionMagic(double position) {
    m_positionMotionMagicVoltage.Slot = 0;
    setControl(m_positionMotionMagicVoltage.withPosition(position));
  }

  /** Tells the motor controller to move to the target position using MotionMagic. */
  public void moveToPositionMotionMagic(double position, double kg, Angle absoluteGravityAngle) {
    var currentRadians = absoluteGravityAngle.in(Radians);
    var currentKg = Math.cos(currentRadians) * kg;
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
