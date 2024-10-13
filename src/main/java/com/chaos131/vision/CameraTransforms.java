package com.chaos131.vision;

import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

import com.chaos131.util.Quad;

/**
 * Series of 3d transformation matrices and data structures to process geometry.
 */
public class CameraTransforms {
    /**
     * Generates a column major view matrix.
     * 
     * @param pose camera pose to derive the view matrix from
     * @return the view matrix
     */
    public static Matrix<N4, N4> ViewMatrix(Pose3d pose) {
        // TODO: Implement me!
        return null;
    }

    /**
     * Generates a column major frustum-projection matrix.
     * 
     * @param n - near plane
     * @param f - far plane
     * @param l - left plane
     * @param r - right plane
     * @param t - top plane
     * @param b - bottom plane
     * @return the frustum projection matrix
     */
    public static Matrix<N4, N4> FrustumProjectionMatrix(double n, double f, double l, double r, double t, double b) {
        // TODO: Implement me!
        return null;
    }

    /**
     * Generates a column major frustum-projection matrix.
     * 
     * @param hfov - Horizontal Field of View in degrees
     * @param vfov - Vertical Field of View in degrees
     * @param n - near plane
     * @param f - far plane
     * @return the frustum projection matrix
     */
    public static Matrix<N4, N4> FrustumProjectionMatrix(double hfov, double vfov, double n, double f) {
        // TODO: Implement me!
        return null;
    }

    /**
     * Overloaded Function for convenience. Assumes units are in meters.
     * 
     * @param pose camera pose, not a robot pose
     * @param quads squares with their model/world transforms already applied
     * @param hfov horizontal field of view (degrees)
     * @param vfov vertical field of view (degrees)
     * @return a list of 4 element vectors that are in view of the camera
     */
    public static ArrayList<Matrix<N4, N1>> CalculateVisibleCoordinates(Pose3d pose, Quad[] quads, double hfov, double vfov) {
        // TODO: Implement me!
        return null;
    }

    /**
     * Calculates each quad's corners in the camera's view.
     * 
     * @param pose camera pose, not a robot pose
     * @param quads squares with their model/world transforms already applied
     * @param hfov horizontal field of view (degrees)
     * @param vfov vertical field of view (degrees)
     * @param n near plane in world units
     * @param f far plane in world units
     * @return a list of 4 element vectors that are in view of the camera
     */
    public static ArrayList<Matrix<N4, N1>> CalculateVisibleCoordinates(Pose3d pose, Quad[] quads, double hfov, double vfov, double n, double f) {
        // TODO: Implement me!
        return null;
    }

    /**
     * Finds the region of interest that contains all of the listed points.
     * 
     * @param points List of vertices that were probably found from CalculateVisibleCoordinates()
     * @param margins horizontal and vertical margins applied to both sides of the region, in camera relative values [-1,1]
     * @return {ll.x, ll.y, ur.x, ur.y}
     */
    public static double[] FindBounds(ArrayList<Matrix<N4, N1>> points, Translation2d margins) {
        // TODO: Implement me!
        return null;
    }
}
