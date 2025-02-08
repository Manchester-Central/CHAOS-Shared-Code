package com.chaos131.vision;

import com.chaos131.util.Quad;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

/**
 * Defines an april tag in the world, and stores information about its id, location, size, and
 * orientation on the field.
 *
 * <p>Really just used for vision systems.
 */
public class AprilTag extends Quad {

  /** The ID number for the April Tag */
  public final int id;

  /** Describes the AprilTag Family, FRC 2024 uses 36h11 */
  public final String family;

  /** Actual pose of the tag in 3d space */
  public final Pose3d pose3d;

  /** Ground pose of the tag on the 2d floor space */
  public final Pose2d pose2d;

  /**
   * April Tag Constructor
   *
   * @param tag_id unique number of the tag
   * @param transform column major matrix with a homogenous coordinate, same coordinate system as
   *     Limelights
   * @param sizeInMeters used to define the distances to the corners from the center of the tag,
   *     assumed to be a square tag
   */
  public AprilTag(
      int tag_id, Matrix<N4, N4> transform, double sizeInMeters, Transform3d coord_shift) {
    this(tag_id, "36h11", transform, sizeInMeters, coord_shift);
  }

  /**
   * April Tag Constructor
   *
   * @param tag_id unique number of the tag
   * @param tag_family describes the code / pattern for the april tag
   * @param transform column major matrix with a homogenous coordinate, same coordinate system as
   *     Limelights
   * @param sizeInMeters used to define the distances to the corners from the center of the tag,
   *     assumed to be a square tag
   */
  public AprilTag(
      int tag_id,
      String tag_family,
      Matrix<N4, N4> transform,
      double sizeInMeters,
      Transform3d coord_shift) {
    super(
        transform.times(VecBuilder.fill(sizeInMeters / -2, 0, sizeInMeters / -2, 1.0)),
        transform.times(VecBuilder.fill(sizeInMeters / 2, 0, sizeInMeters / -2, 1.0)),
        transform.times(VecBuilder.fill(sizeInMeters / 2, 0, sizeInMeters / 2, 1.0)),
        transform.times(VecBuilder.fill(sizeInMeters / -2, 0, sizeInMeters / 2, 1.0)));
    id = tag_id;
    family = tag_family;
    // Calculate the points manually because WPILib can't decode the 4x4 affine matrix into parts
    // Which is uh... not impressive.
    Vector<N3> original_direction = VecBuilder.fill(0, 1, 0);
    Matrix<N4, N1> center_point = transform.times(VecBuilder.fill(0, 0, 0, 1.0));
    Translation3d tag_center =
        new Translation3d(center_point.get(0, 0), center_point.get(1, 0), center_point.get(2, 0));

    Matrix<N4, N1> forward_point = transform.times(VecBuilder.fill(0, 1, 0, 1.0));
    Vector<N3> facing_direction =
        new Vector<N3>(forward_point.minus(center_point).block(3, 1, 0, 0));

    // Why the hell is dot and cross product split between static and member functions?!
    Vector<N3> cross_product = Vector.cross(original_direction, facing_direction);
    Rotation3d rot;
    if (cross_product.norm() < 1e-8) {
      // If the vector is too small we shouldn't use it, so lets default to yaw for now
      double dot_product = original_direction.dot(facing_direction);
      rot = new Rotation3d(0, 0, (1 - dot_product) / 2 * Math.PI);
    } else {
      // The vector is large enough for us to do some stuffs with
      double dot_product = Math.acos(original_direction.dot(facing_direction));
      cross_product = cross_product.div(cross_product.norm()).times(dot_product);
      rot = new Rotation3d(cross_product);
    }

    // Finally... we can set the pose. I still hate EJML and WPILib's math library
    if (coord_shift != null) {
      // Shift from Limelight Coordinate frame to WPIBlue
      pose3d = new Pose3d(tag_center.plus(coord_shift.getTranslation()), rot);
    } else {
      pose3d = new Pose3d(tag_center, rot);
    }
    pose2d = pose3d.toPose2d();
  }
}
