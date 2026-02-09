package com.chaos131.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The state of a module, with logging functions automatically generated with AutoLog. Note that
 * ModuleState should never be directly used, instead use the class 'ModuleStateAutoLogged'.
 *
 * <p>Note that this is separated from the BaseSwerveModule's scope because java does not like the
 * nesting of the derived ModuleStateAutoLogged within an abstract BaseSwerveModule class.
 *
 * @deprecated CHAOS has moved onto using <a
 *     href="https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/">AdvantageKit's
 *     TalonFx template</a>
 */
@AutoLog
@Deprecated(since = "2026.1", forRemoval = true)
public class ModuleState {
  /** Tracks if the motor is active and responsive */
  public boolean driveMotorConnected = true;

  /** Tracks if the motor is active and responsive */
  public boolean turnMotorConnected = true;

  /** Tracks the velocity motor's position in radians */
  public double drivePositionRads = 0.0;

  /** Tracks the velocity motor's rate of rotation */
  public double driveVelocityRadsPerSec = 0.0;

  /** Tracks the velocity motor's ground distance travelled */
  public double driveVelocityMetersPerSec = 0.0;

  /** Tracks the velocity motor's target voltage */
  public double driveAppliedVolts = 0.0;

  /** Tracks the velocity motor's current in amps */
  public double driveSupplyCurrentAmps = 0.0;

  /** Tracks the velocity motor's torque */
  public double driveTorqueCurrentAmps = 0.0;

  /** Tracks the turn motor's position */
  public Rotation2d turnAbsolutePosition = new Rotation2d();

  /** Tracks the turn motor's position */
  public Rotation2d turnPosition = new Rotation2d();

  /** Tracks the turn motor's rate of rotation */
  public double turnVelocityRadsPerSec = 0.0;

  /** Tracks the turn motor's target voltage */
  public double turnAppliedVolts = 0.0;

  /** Tracks the turn motor's current in amps */
  public double turnSupplyCurrentAmps = 0.0;

  /** Tracks the turn motor's torque */
  public double turnTorqueCurrentAmps = 0.0;

  /** A list of signal data from the drive motor */
  public double[] odometryDrivePositionsMeters = new double[] {};

  /** A list of signal data from the turn motor */
  public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
}
