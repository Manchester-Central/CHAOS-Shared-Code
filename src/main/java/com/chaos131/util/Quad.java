package com.chaos131.util;

import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public class Quad {
    public Vector<N4> ll;
    public Vector<N4> lr;
    public Vector<N4> ur;
    public Vector<N4> ul;

    public Quad(Translation3d _ll, Translation3d _lr, Translation3d _ur, Translation3d _ul) {
        ll = VecBuilder.fill(_ll.getX(), _ll.getY(), _ll.getZ(), 1);
        // Implement the rest
    }

    public Quad(Translation3d loc) {
        this(loc.toVector());
    }

    public Quad(Vector<N3> vec) {
        ll = VecBuilder.fill(vec.get(0),vec.get(1),vec.get(2),1);
        // Implement the rest
    }

    public Quad(Matrix<N4, N1> m1, Matrix<N4, N1> m2, Matrix<N4, N1> m3, Matrix<N4, N1> m4) {
        ll = VecBuilder.fill(m1.get(0, 0),m1.get(1, 0),m1.get(2, 0),m1.get(3, 0));
        // Implement the rest
    }

    public Quad() {
        ll = VecBuilder.fill(0,0,0,1);
        // Implement the rest
    }

    public ArrayList<Vector<N4>> getPoints() {
        ArrayList<Vector<N4>> points = new ArrayList<>();
        points.add(ll);
        // Implement the rest
        return null;
    }

    public String toString() {
        return "[" + ll + ", " + lr + ", " + ur + ", " + ul + "]";
    }
}
