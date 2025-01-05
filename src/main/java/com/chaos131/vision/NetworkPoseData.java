package com.chaos131.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * A data structure recording and parsing information from network tables. This doesn't help a
 * camera recalculate pose data from a video feed, however we can re-analyze the pose data. Data
 * stored in here should have the number of timestamps match the number of data blocks (ie, pose
 * data). The individual pose data arrays can be arbitrarily sized to fit whatever Camera type is
 * being used (Limelight, Photon, Whatever).
 */
@AutoLog
public class NetworkPoseData {
  /** Timestamps from NetworkTables, in microseconds */
  public double[] timestamps;

  /** Calculated Pose */
  public Pose3d[] pose;

  /** Number of tags observed in the creation of the pose */
  public int[] tagCount;

  /** Deviation data calculated from the pose metadata */
  public double[] deviations;

  /** Average distance of the tags used for the pose */
  public double[] averageTagDistance;

  /** Resets the data structure to 0-length arrays */
  public void reset() {
    timestamps = new double[] {};
    pose = new Pose3d[] {};
    tagCount = new int[] {};
    deviations = new double[] {};
    averageTagDistance = new double[] {};
  }

  /**
   * Sets the length of every array to a specific size
   *
   * @param s size to set to
   */
  public void resize(int s) {
    timestamps = new double[s];
    pose = new Pose3d[s];
    tagCount = new int[s];
    deviations = new double[s];
    averageTagDistance = new double[s];
  }
}
