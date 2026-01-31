package com.chaos131.poses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public abstract class FieldPose {
  public static final Map<String, FieldPose> FieldPoses = new HashMap<String, FieldPose>();

  /** The pre-calculated red pose */
  protected Pose3d m_redPose;

  /** The pre-calculated blue pose */
  protected Pose3d m_bluePose;

  /** A potentially null name for the pose */
  protected final String m_name;

  /** The corresponding alliance the original pose was built for */
  protected final Alliance m_defaultAlliance;

  public FieldPose(
      Translation2d midpoint, Alliance defaultAlliance, String name, Pose3d defaultPose) {
    m_defaultAlliance = defaultAlliance;
    m_name = name;

    if (defaultAlliance == Alliance.Blue) {
      m_bluePose = defaultPose;
      m_redPose = calculateSymmetry(midpoint, defaultPose);
    } else {
      m_bluePose = calculateSymmetry(midpoint, defaultPose);
      m_redPose = defaultPose;
    }

    // only register drive poses with a name
    if (name != null && !name.trim().isEmpty()) {
      FieldPoses.put(name, this);
    }
  }

  /**
   * @return the blue alliance's pose
   */
  public Pose2d getBluePose() {
    return m_bluePose.toPose2d();
  }

  public Pose3d getBluePose3d() {
    return m_bluePose;
  }

  /**
   * @return the red alliance's pose
   */
  public Pose2d getRedPose() {
    return m_redPose.toPose2d();
  }

  public Pose3d getRedPose3d() {
    return m_redPose;
  }

  /**
   * @return the name for the pose
   */
  public String getName() {
    return m_name;
  }

  public abstract Pose3d calculateSymmetry(Translation2d midpoint, Pose3d pose);

  /**
   * Calculates the distance from a specific spot on the field. Typically this is the robot's
   * position.
   *
   * @param robotpose to calculate from
   * @return the distance along the floor. Does not account for vertical component.
   */
  public Distance getDistanceFromLocation(Pose2d robotpose) {
    return getDistanceFromLocations(getCurrentAlliancePose(), robotpose);
  }

  /**
   * Calculates the distance from a specific spot on the field. Typically this is the robot's
   * position.
   *
   * @param pose1
   * @param pose2 
   * @return the distance along the floor. Does not account for vertical component. Returns absolute values.
   */
  public static Distance getDistanceFromLocations(Pose2d pose1, Pose2d pose2) {
    return Meters.of(pose2.getTranslation().getDistance(pose1.getTranslation()));
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
    return getCurrentAlliance() == Alliance.Blue ? getBluePose() : getRedPose();
  }

  /**
   * Gets the closes known pose in FieldPoses
   *
   * @param robotPose the pose to find a FieldPose near
   * @return the closest named pose
   */
  public static Pose2d getClosestKnownPose(Pose2d robotPose) {
    var poses = new ArrayList<Pose2d>();
    FieldPoses.forEach((name, pose) -> poses.add(pose.getCurrentAlliancePose()));
    return robotPose.nearest(poses);
  }
}
