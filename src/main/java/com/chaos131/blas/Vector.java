package com.chaos131.blas;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Dynamically sizeable Vector class that ports into and out of FRC data structures
 */
public class Vector implements Interpolatable<Vector> {
	// Variable sized native array. Assumed to be terms in the x, y, z, w ordering.
	protected double[] m_values;

	public Vector(Translation2d t) {
		m_values = new double[]{t.getX(), t.getY()};
	}

	public Vector(Translation3d t) {
		m_values = new double[]{t.getX(), t.getY(), t.getZ()};
	}

	public Vector(double x, double y) {
		m_values = new double[]{x, y};
	}

	public Vector(double[] v) {
		m_values = v;
	}

	public Vector(Vector v) {
		System.arraycopy( v.m_values, 0, m_values, 0, v.m_values.length );
	}

	public Translation2d toTranslation2d() {
		return new Translation2d(m_values[0], m_values[1]);
	}

	public double[] getValues() {
		return m_values;
	}

	public int getSize() {
		return m_values.length;
	}

	public double at(int i) {
		// This could throw an index out of bounds exception!
		return m_values[i];
	}

	public void set(int idx, double val) {
		// This could throw an index out of bounds exception!
		m_values[idx] = val;
	}

	public double mag() {
		double sum = 0;
		for (int i = 0; i < m_values.length; i++) {
			sum += m_values[i] * m_values[i];
		}
		return sum;
	}

	/**
	 * Adds two vectors together element wise.
	 * 
	 * @param v - A vector of the same size
	 * @return
	 */
	public Vector add(Vector v) {
		if (m_values.length != v.getSize()) {
			return null;
		}
		double[] out = new double[m_values.length];
		for (int i = 0; i < m_values.length; i++) {
			out[i] = v.at(i) + m_values[i];
		}
		return new Vector(out);
	}

	/**
	 * Subtracts two vectors element wise.
	 * 
	 * @param v - A vector of the same size
	 * @return
	 */
	public Vector subtract(Vector v) {
		if (m_values.length != v.getSize()) {
			return null;
		}
		double[] out = new double[m_values.length];
		for (int i = 0; i < m_values.length; i++) {
			out[i] = m_values[i] - v.at(i);
		}
		return new Vector(out);
	}

	/**
	 * Cross Product of the two vectors. Both vectors must be 3 elements long.
	 * 
	 * @param v - vector
	 * @return
	 */
	public Vector cross(Vector v) {
		if (m_values.length != 3 && v.getSize() != 3) {
			return null;
		}
		double[] out = new double[3];
		return new Vector(out);
	}

	/**
	 * Dot product of the two vectors. Both vectors must be the same size.
	 * 
	 * @param v - vector
	 * @return
	 */
	public double dot(Vector v) {
		double sum = 0;
		for (int i = 0; i < m_values.length; i++) {
			sum += m_values[i] * v.at(i);
		}
		return sum;
	}

	/**
	 * Element-wise multiplication.
	 * 
	 * Both vectors must be the same size, and the returned vector is the same size as them.
	 * Note that there are multiple ways to do vector-vector multiplication.
	 * 
	 * @param v - vector
	 * @return
	 */
	public Vector mult(Vector v) {
		if (m_values.length != v.getSize()) {
			return null;
		}
		double[] out = new double[m_values.length];
		for (int i = 0; i < m_values.length; i++) {
			out[i] = v.at(i) * m_values[i];
		}
		return new Vector(out);
	}

	/**
	 * Scalar multiplication. Pretty self explanatory.
	 * 
	 * @param s - double
	 * @return
	 */
	public Vector mult(double s) {
		double[] out = new double[m_values.length];
		for (int i = 0; i < m_values.length; i++) {
			out[i] = s * m_values[i];
		}
		return new Vector(out);
	}

	/**
	 * Lerps the value between two points.
	 * 0 is the original point, to 1 the end point.
	 * 
	 * @param endValue
	 * @param t
	 * @return
	 */
	@Override
	public Vector interpolate(Vector endValue, double t) {
		return mult(1-t).add(endValue.mult(t));
	}
}
