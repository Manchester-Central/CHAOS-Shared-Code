// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.chaos131.poses;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

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
public abstract class MirroredDrivePose {

  /**
   * A list of all the registed drive poses. This lets you create commands for driving to all known
   * poses or specifying drive poses in auto scripts
   */
  public static final Map<String, MirroredDrivePose> DrivePoses =
      new HashMap<String, MirroredDrivePose>();

  /** The center line along the x coordinate */
  protected final double m_fieldCenterPointMeters;

  /** The pre-calculated red pose */
  protected final Pose2d m_redPose;

  /** The pre-calculated blue pose */
  protected final Pose2d m_bluePose;

  /** A potentially null name for the pose */
  protected final String m_name;

  /** The corresponding alliance the original pose was built for */
  protected final Alliance m_defaultAlliance;

  /**
   * Creates a new drive pose with mirrored red and blue poses.
   *
   * @param fieldWidthMeters the width of the field you want to mirror on. (With field width 10, an
   *     x of 0 for blue will map to an x of 10 for red)
   * @param defaultAlliance the alliance color that will be on the (0, 0) side of your field
   * @param name the name of the pose. Leave as null if you don't want to add it to DrivePoses
   * @param defaultPose the pose to mirror
   */
  protected MirroredDrivePose(
      double fieldWidthMeters, Alliance defaultAlliance, String name, Pose2d defaultPose) {
    m_fieldCenterPointMeters = fieldWidthMeters / 2;
    m_defaultAlliance = defaultAlliance;
    m_name = name;

    if (defaultAlliance == Alliance.Blue) {
      m_bluePose = defaultPose;
      m_redPose = mirrorPoseOnField(m_fieldCenterPointMeters, defaultPose);
    } else {
      m_bluePose = mirrorPoseOnField(m_fieldCenterPointMeters, defaultPose);
      m_redPose = defaultPose;
    }

    // only register drive poses with a name
    if (name != null && !name.trim().isEmpty()) {
      DrivePoses.put(name, this);
    }
  }

  /**
   * Creates a new drive pose with mirrored red and blue poses
   *
   * @param fieldWidthMeters the width of the field you want to mirror on. (With field width 10, an
   *     x of 0 for blue will map to an x of 10 for red)
   * @param defaultAlliance the alliance color that will be on the (0, 0) side of your field
   * @param name the name of the pose. Leave as null if you don't want to add it to DrivePoses
   * @param defaultXMeters the x position
   * @param defaultYMeters the y position
   * @param defaultAngle the angle
   */
  protected MirroredDrivePose(
      double fieldWidthMeters,
      Alliance defaultAlliance,
      String name,
      double defaultXMeters,
      double defaultYMeters,
      Rotation2d defaultAngle) {
    this(
        fieldWidthMeters,
        defaultAlliance,
        name,
        new Pose2d(defaultXMeters, defaultYMeters, defaultAngle));
  }

  /**
   * @return the blue alliance's pose
   */
  public Pose2d getBluePose() {
    return m_bluePose;
  }

  /**
   * @return the red alliance's pose
   */
  public Pose2d getRedPose() {
    return m_redPose;
  }

  /**
   * @return the name for the pose
   */
  public String getName() {
    return m_name;
  }

  /**
   * Calculates the distance from a specific spot on the field. Typically this is the robot's
   * position.
   *
   * @param robotpose to calculate from
   * @return the distance along the floor. Does not account for vertical component.
   */
  public double getDistanceFromLocation(Pose2d robotpose) {
    if (getCurrentAlliance() == Alliance.Blue) {
      return m_bluePose.getTranslation().getDistance(robotpose.getTranslation());
    } else {
      return m_redPose.getTranslation().getDistance(robotpose.getTranslation());
    }
  }

  /**
   * @param robotpose to calculate from
   * @return Rotation2d - the field coordinate angle relative to field angle 0.
   */
  public Rotation2d angleFrom(Translation2d robotpose) {
    return getCurrentAlliancePose().getTranslation().minus(robotpose).getAngle();
  }

  /**
   * Gets the appropriate pose for the current alliance color. (Override this when using unit
   * testing since the call to DriverStation will throw an exception)
   *
   * @return the current alliance
   */
  protected Alliance getCurrentAlliance() {
    return DriverStation.getAlliance().orElse(m_defaultAlliance);
  }

  /**
   * Gets the pose for the robot's current alliance. (If the DS is not connected, the default
   * alliance is used) By default, the pose is initialized for blue, and will mirror the pose to red
   * when on the red alliance.
   *
   * @return pose for the target, mirrored if appropriate
   */
  public Pose2d getCurrentAlliancePose() {
    return getCurrentAlliance() == Alliance.Blue ? m_bluePose : m_redPose;
  }

  /**
   * Gets the closes known pose in DrivePoses
   *
   * @param robotPose the pose to find a DrivePose near
   * @return the closest named pose
   */
  public static Pose2d getClosestKnownPose(Pose2d robotPose) {
    var poses = new ArrayList<Pose2d>();
    DrivePoses.forEach((name, pose) -> poses.add(pose.getCurrentAlliancePose()));
    ;
    return robotPose.nearest(poses);
  }

  /**
   * Mirrors a Pose2d on the field by flipping across a line across midfield.
   *
   * <p>This changes both the translation and rotation components.
   *
   * @param fieldCenterPointMeters the center line to mirror the x value on
   * @param pose the pose to mirror
   * @return the new mirrored pose
   */
  public static Pose2d mirrorPoseOnField(double fieldCenterPointMeters, Pose2d pose) {
    var xDistanceToMid = fieldCenterPointMeters - pose.getX();
    var newX = fieldCenterPointMeters + xDistanceToMid;
    var newY = pose.getY();
    var newAngle = Rotation2d.fromDegrees(180).minus(pose.getRotation());
    return new Pose2d(newX, newY, newAngle);
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
  public static MirroredDrivePose makeWPIBluePoseFromCenteredPose(
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
