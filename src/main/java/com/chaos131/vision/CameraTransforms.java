package com.chaos131.vision;

import com.chaos131.util.Quad;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import java.util.ArrayList;
import org.ejml.simple.SimpleMatrix;

/** Series of 3d transformation matrices and data structures to process geometry. */
public class CameraTransforms {
  /**
   * Generates a column major view matrix.
   *
   * @param pose camera pose to derive the view matrix from
   * @return the view matrix
   */
  public static Matrix<N4, N4> ViewMatrix(Pose3d pose) {
    var trans =
        MatBuilder.fill(
            Nat.N4(),
            Nat.N4(),
            new double[] {
              1, 0, 0, -pose.getX(),
              0, 1, 0, -pose.getY(),
              0, 0, 1, -pose.getZ(),
              0, 0, 0, 1
            });
    // x axis
    var forward = new Translation3d(1, pose.getRotation()).toVector();
    // y axis
    var right = Vector.cross(forward, new Vector<>(new SimpleMatrix(new double[] {0, 0, 1})));
    right = right.div(right.norm());
    // z axis
    var up = Vector.cross(right, forward);
    return MatBuilder.fill(
            Nat.N4(),
            Nat.N4(),
            new double[] {
              right.get(0),
              right.get(1),
              right.get(2),
              0,
              up.get(0),
              up.get(1),
              up.get(2),
              0,
              -forward.get(0),
              -forward.get(1),
              -forward.get(2),
              0,
              0,
              0,
              0,
              1
            })
        .times(trans);
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
  public static Matrix<N4, N4> FrustumProjectionMatrix(
      double n, double f, double l, double r, double t, double b) {
    return MatBuilder.fill(
        Nat.N4(),
        Nat.N4(),
        new double[] {
          (2 * n / (r - l)),
          0,
          (r + l) / (r - l),
          0,
          0,
          (2 * n / (t - b)),
          (t + b) / (t - b),
          0,
          0,
          0,
          -(f + n) / (f - n),
          -(2 * f * n) / (f - n),
          0,
          0,
          -1,
          0
        });
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
  public static Matrix<N4, N4> FrustumProjectionMatrix(
      double hfov, double vfov, double n, double f) {
    var h = 1 / Math.tan(hfov * Math.PI / 360.0); // simplified from (hfov/2)*(pi/180)
    var v = 1 / Math.tan(vfov * Math.PI / 360.0);
    return MatBuilder.fill(
        Nat.N4(),
        Nat.N4(),
        new double[] {h, 0, 0, 0, 0, v, 0, 0, 0, 0, -f / (f - n), -f * n / (f - n), 0, 0, -1, 0});
  }

  /**
   * Overloaded Function for convenience.
   *
   * @param pose camera pose, not a robot pose
   * @param quads squares with their model/world transforms already applied
   * @param hfov horizontal field of view (degrees)
   * @param vfov vertical field of view (degrees)
   * @return a list of 4 element vectors that are in view of the camera
   */
  public static ArrayList<Matrix<N4, N1>> CalculateVisibleCoordinates(
      Pose3d pose, Quad[] quads, double hfov, double vfov) {
    return CalculateVisibleCoordinates(pose, quads, hfov, vfov, 0.05, 100);
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
  public static ArrayList<Matrix<N4, N1>> CalculateVisibleCoordinates(
      Pose3d pose, Quad[] quads, double hfov, double vfov, double n, double f) {
    var view_projection_matrix = FrustumProjectionMatrix(hfov, vfov, n, f).times(ViewMatrix(pose));
    ArrayList<Matrix<N4, N1>> final_points = new ArrayList<>();
    for (Quad q : quads) {
      var points = q.getPoints();
      for (int idx = 0; idx < points.size(); idx++) {
        var p = view_projection_matrix.times(points.get(idx));
        p = p.div(p.get(3, 0)); // make sure the homogenous coordinate is normalized
        // System.out.println(p);
        if (-1 < p.get(0, 0)
            && p.get(0, 0) < 1
            && -1 < p.get(1, 0)
            && p.get(1, 0) < 1
            && -1 < p.get(2, 0)
            && p.get(2, 0) < 1) {
          final_points.add(p);
        }
      }
    }
    return final_points;
  }

  /**
   * Calculates each quad's corners in the camera's view.
   *
   * @param pose camera pose, not a robot pose
   * @param quads squares with their model/world transforms already applied
   * @param view_proj the view projection matrix (calculated with FrustumProjectionMatrix)
   * @return a list of 4 element vectors that are in view of the camera
   */
  public static ArrayList<Matrix<N4, N1>> CalculateVisibleCoordinates(
      Pose3d pose, Quad[] quads, Matrix<N4, N4> view_proj) {
    ArrayList<Matrix<N4, N1>> final_points = new ArrayList<>();
    for (Quad q : quads) {
      var points = q.getPoints();
      for (int idx = 0; idx < points.size(); idx++) {
        var p = view_proj.times(points.get(idx));
        p = p.div(p.get(3, 0)); // make sure the homogenous coordinate is normalized
        // System.out.println(p);
        if (-1 < p.get(0, 0)
            && p.get(0, 0) < 1
            && -1 < p.get(1, 0)
            && p.get(1, 0) < 1
            && -1 < p.get(2, 0)
            && p.get(2, 0) < 1) {
          final_points.add(p);
        }
      }
    }
    return final_points;
  }

  /**
   * Finds the region of interest that contains all of the listed points.
   *
   * @param points List of vertices that were probably found from CalculateVisibleCoordinates()
   * @param horizontal_margin horizontal and vertical margins applied to both sides of the region,
   *     in camera relative values [-1,1]
   * @param vertical_margin See above.
   * @return {ll.x, ll.y, ur.x, ur.y}
   */
  public static double[] FindBounds(
      ArrayList<Matrix<N4, N1>> points, double horizontal_margin, double vertical_margin) {
    Double llx = null;
    Double lly = null;
    Double urx = null;
    Double ury = null;

    for (var p : points) {
      if (llx == null || p.get(0, 0) < llx) {
        llx = p.get(0, 0);
      }
      if (urx == null || urx < p.get(0, 0)) {
        urx = p.get(0, 0);
      }
      if (lly == null || p.get(1, 0) < lly) {
        lly = p.get(1, 0);
      }
      if (ury == null || ury < p.get(1, 0)) {
        ury = p.get(1, 0);
      }
    }

    if (llx == null || urx == null || lly == null || ury == null) {
      return null;
    }

    // return new Double[] { llx, lly, urx, ury };
    return new double[] {
      Math.max(lly - horizontal_margin, -1.0),
      Math.max(lly - vertical_margin, -1.0),
      Math.min(urx + horizontal_margin, 1.0),
      Math.min(ury + vertical_margin, 1.0)
    };
  }
}
