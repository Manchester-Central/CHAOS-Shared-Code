package com.chaos131.blas;

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
	 * Finds the point between the current place, and the target to launch the projectile.
	 * 
	 * @param p0
	 * @param v0
	 * @param target
	 * @param acceleration
	 * @return
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

		var plumb_dist_mag = plumb_distance.mag();
		var velocity_mag = v0.mag();

		

		return null;
	}
}
