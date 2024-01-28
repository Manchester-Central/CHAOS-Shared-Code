package com.chaos131.util;

import edu.wpi.first.math.controller.proto.ElevatorFeedforwardProto;
import edu.wpi.first.math.controller.struct.ElevatorFeedforwardStruct;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * A helper class that computes feedforward outputs for a simple elevator (modeled as a motor acting
 * against the force of gravity).
 */
public class FullFeedForward implements ProtobufSerializable, StructSerializable {
	/** The static gain. */
	public double ks;

	/** The gravity gain. */
	public double kg;

	/** The velocity gain. */
	public double kv;

	/** The acceleration gain. */
	public double ka;

	// TODO: This should probably evolve a bit, and have its own ProtoBufs

	/** ElevatorFeedforward protobuf for serialization. */
	public static final ElevatorFeedforwardProto proto = new ElevatorFeedforwardProto();

	/** ElevatorFeedforward struct for serialization. */
	public static final ElevatorFeedforwardStruct struct = new ElevatorFeedforwardStruct();

	/**
	 * All calculations are done assuming the angles are done with respect to the horizon.
	 * 
	 * Therefore, full gravity with be with a pitch of 0 radians (sticking straight out),
	 * and no gravity is treated as half-pi radians (90 degrees up).
	 * Observe that a negative angle would produce the same result.
	 */
	public static final Rotation3d FULL_GRAVITY = new Rotation3d(0, 0, 0);
	public static final Rotation3d NO_GRAVITY = new Rotation3d(0, Math.PI/2, 0);

	/**
	 * Velocity / Driving based Feed Forward calculator.
	 * Useful for when you need to know the speed of the wheel.
	 */
	public static class VelocityFeedForward extends FullFeedForward {
		public VelocityFeedForward(double ks, double kv, double ka) {
			super(ks, kv, ka, 0);
		}
	}
	/**
	 * Useful for rotating mechanisms, like an arm.
	 */
	public class AngularFeedForward extends FullFeedForward {
		public AngularFeedForward(double ks, double kv, double ka, double kg) {
			super(ks, kv, ka, kg);
		}
	}
	/**
	 * Useful for when there's some external acceleration component.
	 * Typically this is due to gravity, for things like elevators and claws.
	 */
	public class GravitationFeedForward extends FullFeedForward {
		public GravitationFeedForward(double ks, double kv, double ka, double kg) {
			super(ks, kv, ka, kg);
		}
	}

	/**
	 * Creates a new ElevatorFeedforward with the specified gains. Units of the gain values will
	 * dictate units of the computed feedforward.
	 *
	 * @param ks The static gain.
	 * @param kv The velocity gain.
	 * @param ka The acceleration gain.
	 * @param kg The gravity gain.
	 * @throws IllegalArgumentException for kv &lt; zero.
	 * @throws IllegalArgumentException for ka &lt; zero.
	 */
	public FullFeedForward(double ks, double kv, double ka, double kg) {
		this.ks = ks;
		this.kg = kg;
		this.kv = kv;
		this.ka = ka;
		if (kv < 0.0) {
			throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
		}
		if (ka < 0.0) {
			throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
		}
	}

	public void updateValues(double s, double v, double a, double g) {
		ks = s;
		kv = v;
		ka = a;
		kg = g;
	}

	/**
	 * Calculates the feedforward from the gains and setpoints.
	 *
	 * @param velocity The velocity setpoint.
	 * @param acceleration The acceleration setpoint.
	 * @return The computed feedforward.
	 */
	public double calculate(double velocity, double acceleration, Rotation3d rot) {
		return ks * Math.signum(velocity) 
			+ kg * Math.cos(rot.getY())
			+ kv * velocity 
			+ ka * acceleration;
	}

	/**
	 * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be
	 * zero).
	 *
	 * @param velocity The velocity setpoint.
	 * @return The computed feedforward.
	 */
	public double calculate(double velocity) {
		return calculate(velocity, 0, new Rotation3d());
	}

	// Rearranging the main equation from the calculate() method yields the
	// formulas for the methods below:

	/**
	 * Calculates the maximum achievable velocity given a maximum voltage supply and an acceleration.
	 * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
	 * simultaneously achievable - enter the acceleration constraint, and this will give you a
	 * simultaneously-achievable velocity constraint.
	 *
	 * @param maxVoltage The maximum voltage that can be supplied to the elevator.
	 * @param acceleration The acceleration of the elevator.
	 * @return The maximum possible velocity at the given acceleration.
	 */
	public double maxAchievableVelocity(double maxVoltage, double acceleration, Rotation3d rot) {
		// Assume max velocity is positive
		return (maxVoltage - ks - Math.cos(rot.getY()) * kg - acceleration * ka) / kv;
	}

	/**
	 * Calculates the minimum achievable velocity given a maximum voltage supply and an acceleration.
	 * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
	 * simultaneously achievable - enter the acceleration constraint, and this will give you a
	 * simultaneously-achievable velocity constraint.
	 *
	 * @param maxVoltage The maximum voltage that can be supplied to the elevator.
	 * @param acceleration The acceleration of the elevator.
	 * @return The minimum possible velocity at the given acceleration.
	 */
	public double minAchievableVelocity(double maxVoltage, double acceleration, Rotation3d rot) {
		// Assume min velocity is negative, ks flips sign
		return (-maxVoltage + ks - Math.cos(rot.getY()) * kg - acceleration * ka) / kv;
	}

	/**
	 * Calculates the maximum achievable acceleration given a maximum voltage supply and a velocity.
	 * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
	 * simultaneously achievable - enter the velocity constraint, and this will give you a
	 * simultaneously-achievable acceleration constraint.
	 *
	 * @param maxVoltage The maximum voltage that can be supplied to the elevator.
	 * @param velocity The velocity of the elevator.
	 * @return The maximum possible acceleration at the given velocity.
	 */
	public double maxAchievableAcceleration(double maxVoltage, double velocity, Rotation3d rot) {
		return (maxVoltage - ks * Math.signum(velocity) - Math.cos(rot.getY()) * kg - velocity * kv) / ka;
	}

	/**
	 * Calculates the minimum achievable acceleration given a maximum voltage supply and a velocity.
	 * Useful for ensuring that velocity and acceleration constraints for a trapezoidal profile are
	 * simultaneously achievable - enter the velocity constraint, and this will give you a
	 * simultaneously-achievable acceleration constraint.
	 *
	 * @param maxVoltage The maximum voltage that can be supplied to the elevator.
	 * @param velocity The velocity of the elevator.
	 * @return The minimum possible acceleration at the given velocity.
	 */
	public double minAchievableAcceleration(double maxVoltage, double velocity, Rotation3d rot) {
		return maxAchievableAcceleration(-maxVoltage, velocity, rot);
	}
}
