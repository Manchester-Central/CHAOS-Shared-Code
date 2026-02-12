package com.chaos131.poses;

import static edu.wpi.first.units.Units.Degrees;

import com.chaos131.vision.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PivotedFieldPose extends FieldPose {
  protected PivotedFieldPose(
      Translation2d midpoint, Alliance defaultAlliance, String name, Pose3d defaultPose) {
    super(midpoint, defaultAlliance, name, defaultPose);
  }

  protected PivotedFieldPose(
      Translation2d midpoint, Alliance defaultAlliance, String name, Pose2d defaultPose) {
    super(midpoint, defaultAlliance, name, new Pose3d(defaultPose));
  }

  /**
   * Pose generator for use with April Tags which aren't associated with an alliance. There's room
   * for improvement here to associate an alliance with an april tag, but the data currently isn't
   * in the fmap.
   *
   * @param midpoint dead center of the field that the pose will rotate around, can take half of the
   *     fields length and width for this
   * @param tag_id the int id of the april tag
   * @param transform 4x4 matrix that defines the affine transformation
   */
  protected PivotedFieldPose(Translation2d midpoint, int tag_id, Matrix<N4, N4> transform) {
    super(
        midpoint,
        Alliance.Blue,
        "Tag" + tag_id,
        new AprilTag(
                tag_id,
                transform,
                0.0,
                new Transform3d(new Transform2d(midpoint, Rotation2d.kZero)))
            .pose3d);
  }

  public Pose3d calculateSymmetry(Translation2d midpoint, Pose3d pose) {
    // Probably not necessary to raise the point, but do it just in case
    Translation3d rotationPoint =
        new Translation3d(midpoint.getX(), midpoint.getY(), pose.getTranslation().getZ());
    return pose.rotateAround(
        rotationPoint, new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180)));
  }
}
