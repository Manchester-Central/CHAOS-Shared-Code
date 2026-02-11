// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.poses;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A class for managing drive poses and mirroring (x and angle flipped, but y is the same) them for
 * different sides of the field.
 *
 * <p>In your project, you should create your own class that extends it for your expected field
 * width:
 *
 * <pre>
 * class DrivePose2024 extends MirroredDrivePose {
 * public static final double FieldWidthMeters = 16.57;
 * public static final Alliance DefaultAlliance = Alliance.Blue;
 *
 * public DrivePose2024(String name, Pose2d bluePose) {
 * super(FieldWidthMeters, DefaultAlliance, name, bluePose);
 * }
 *
 * public DrivePose2024(Pose2d bluePose) {
 * super(FieldWidthMeters, DefaultAlliance, null, bluePose);
 * }
 * }
 * </pre>
 */
public abstract class MirroredFieldPose extends FieldPose {
  /**
   * Creates a new drive pose with mirrored red and blue poses.
   *
   * @param midpoint the center of the field - use the field length and width and divide by 2
   * @param defaultAlliance the alliance color that will be on the (0, 0) side of your field
   * @param name the name of the pose. Leave as null if you don't want to add it to DrivePoses
   * @param defaultPose the pose to mirror
   */
  protected MirroredFieldPose(
      Translation2d midpoint, Alliance defaultAlliance, String name, Pose3d defaultPose) {
    super(midpoint, defaultAlliance, name, defaultPose);
  }

  protected MirroredFieldPose(
      Translation2d midpoint, Alliance defaultAlliance, String name, Pose2d defaultPose) {
    this(midpoint, defaultAlliance, name, new Pose3d(defaultPose));
  }

  /**
   * Pose generator for use with April Tags which aren't associated with an alliance. There's room
   * for improvement here to associate an alliance with an april tag, but the data currently isn't
   * in the fmap.
   *
   * @param midpoint
   * @param name
   * @param transform
   */
  protected MirroredFieldPose(Translation2d midpoint, String name, Matrix<N4, N4> transform) {
    super(midpoint, Alliance.Blue, name, new Pose3d().transformBy(new Transform3d(transform)));
  }

  /**
   * Creates a new drive pose with mirrored red and blue poses
   *
   * @param midpoint the center of the field - use the field length and width and divide by 2
   * @param defaultAlliance the alliance color that will be on the (0, 0) side of your field
   * @param name the name of the pose. Leave as null if you don't want to add it to DrivePoses
   * @param defaultXMeters the x position
   * @param defaultYMeters the y position
   * @param defaultAngle the angle
   */
  protected MirroredFieldPose(
      Translation2d midpoint,
      Alliance defaultAlliance,
      String name,
      double defaultXMeters,
      double defaultYMeters,
      Rotation2d defaultAngle) {
    this(midpoint, defaultAlliance, name, new Pose2d(defaultXMeters, defaultYMeters, defaultAngle));
  }

  /**
   * Mirrors a Pose2d on the field by flipping across a line across midfield.
   *
   * <p>This changes both the translation and rotation components.
   *
   * @param midpoint the center of the field - use the field length and width and divide by 2
   * @param pose the pose to mirror
   * @return the new mirrored pose
   */
  public Pose3d calculateSymmetry(Translation2d midpoint, Pose3d pose) {
    Pose2d flattenedPose = pose.toPose2d();
    var xDistanceToMid = midpoint.getX() - flattenedPose.getX();
    var newX = midpoint.getX() + xDistanceToMid;
    var newY = flattenedPose.getY();
    var newAngle = Rotation2d.fromDegrees(180).minus(flattenedPose.getRotation());
    return new Pose3d(newX, newY, pose.getTranslation().getZ(), new Rotation3d(newAngle));
  }

  /**
   * UNIMPLEMENTED - For Students to complete!
   *
   * <p>Creates a MirroredDrivePose from
   *
   * @param pt The point in 4 part vector form (x, y, z, w)
   * @param fieldWidthMeters the width of the field (typically the shorter length)
   * @param fieldHeightMeters the length of the field (typically the longer length)
   * @param defaultAlliance the alliance the pose corresponds to
   * @param name of the pose
   * @return the constructed MirroredDrivePose
   */
  public static MirroredFieldPose makeWPIBluePoseFromCenteredPose(
      Matrix<N4, N1> pt,
      double fieldWidthMeters,
      double fieldHeightMeters,
      Alliance defaultAlliance,
      String name) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'makeWPIBluePoseFromCenteredPose'");
  }
}
