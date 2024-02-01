package com.chaos131.blas;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Trajectory {
	/**
	 * Calculates the location of a projectile over some amount of time.
	 * Note that every Vector must be the same size as the others, using the same units.
	 * 
	 * IMPORTANT - The acceleration component *must* be in vector form. This is unusual for high school
	 * education, but should be easily explained.
	 * 
	 * @param p0 - Initial location
	 * @param v0 - velocity vector
	 * @param acceleration - a vector of acceleration components
	 * @param time - 
	 * @return
	 */
	static public Vector calculateProjectileOverTime(Vector p0, Vector v0, Vector acceleration, double time) {
		return p0.add(v0.mult(time)).add(acceleration.mult(time*time).mult(0.5));
	}

	/**
	 * Calculates where the ray would intersect the floor
	 */
	static public Translation3d traceToFloor(Vector ray, Pose3d location) {
		double vertical_distance = location.getZ()/ray.m_values[2];
		return new Translation3d(ray.m_values[0] * vertical_distance, ray.m_values[1] * vertical_distance, 0);
	}

	/**
	 * Finds the point between the current place, and the target to launch the projectile.
	 * 
	 * @param p0 - Location of the launcher relative to the floor
	 * @param v0 - velocity of the launched projectile
	 * @param target - 3d location on the game field of the target spot, using blue alliance field coordinates
	 * @param acceleration - acceleration in every component
	 * @return 2 element launch point
	 */
	static public Vector findIdealLaunchPlace(Vector p0, Vector v0, Vector target, Vector acceleration, int idx) {
		// ax^2 + bx + c = 0
		// 0.5*acc_y*t^2 + v_y*t + delta_y = 0
		// [-b +/- sqrt(b*b - 4ac)] / 2a
		var a = acceleration.at(idx);
		var b = v0.at(idx);
		var c = target.at(idx) - p0.at(idx);

		var sq_root = b*b - 4 * a * c;
		if ( sq_root < 0 ) return null; // no roots to be found
		sq_root = Math.sqrt(sq_root);

		var time_to_peak_1 = (-b + sq_root) / (2 * a);
		var time_to_peak_2 = (-b - sq_root) / (2 * a);

		var first_peak = (time_to_peak_1 < time_to_peak_2 && 0 < time_to_peak_1) ? time_to_peak_1 : time_to_peak_2;
		if (first_peak < 0) return null; // we only have negative timestamps, so we're probably in reverse?

		// now we take the time, and figure out the distance

		var plumb_distance = target.subtract(v0);	// subtract the location from the distance
		plumb_distance.set(idx, 0);				// then zero out the vertical component

		var plumb_dist_mag = plumb_distance.magnitude();
		var velocity_mag = v0.magnitude();

		

		return null;
	}
}
