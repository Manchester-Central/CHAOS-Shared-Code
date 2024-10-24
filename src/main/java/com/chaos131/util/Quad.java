package com.chaos131.util;

import java.util.ArrayList;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

/**
 * A 3d shape with 4 corners, useful for 
 */
public class Quad {
    /** Lower Left corner */
    public Vector<N4> ll;
    /** Lower Right corner */
    public Vector<N4> lr;
    /** Upper Right corner */
    public Vector<N4> ur;
    /** Upper Left corner */
    public Vector<N4> ul;

    /**
     * Makes a quad from 4 unique points
     * @param _ll lower left corner
     * @param _lr lower right corner
     * @param _ur upper right corner
     * @param _ul upper left corner
     */
    public Quad(Translation3d _ll, Translation3d _lr, Translation3d _ur, Translation3d _ul) {
        this(_ll.toVector(), _lr.toVector(), _ur.toVector(), _ul.toVector());
    }

    /**
     * Makes a quad from 4 unique points
     * @param _ll lower left corner
     * @param _lr lower right corner
     * @param _ur upper right corner
     * @param _ul upper left corner
     */
    public Quad(Vector<N3> _ll, Vector<N3> _lr, Vector<N3> _ur, Vector<N3> _ul) {
        ll = VecBuilder.fill(_ll.get(0),_ll.get(1),_ll.get(2), 1);
        lr = VecBuilder.fill(_lr.get(0),_lr.get(1),_lr.get(2), 1);
        ul = VecBuilder.fill(_ul.get(0),_ul.get(1),_ul.get(2), 1);
        ur = VecBuilder.fill(_ur.get(0),_ur.get(1),_ur.get(2), 1);
    }

    /**
     * @return all 4 corners into an ArrayList
     */
    public ArrayList<Vector<N4>> getPoints() {
        ArrayList<Vector<N4>> points = new ArrayList<>();
        points.add(ll);
        points.add(lr);
        points.add(ul);
        points.add(ur);
        return points;
    }

    /**
     * @return returns the coordinates in string form
     */
    public String toString() {
        return "[" + ll + ", " + lr + ", " + ur + ", " + ul + "]";
    }
}
