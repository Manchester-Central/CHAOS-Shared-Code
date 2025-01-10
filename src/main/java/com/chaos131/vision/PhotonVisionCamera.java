package com.chaos131.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** An unimplemented class to process PhotonVision Updates */
public class PhotonVisionCamera extends Camera {
  /** The PhotonCamera class from PhotonLib that we wrap up */
  protected PhotonCamera m_camera;

  /**
   * Still unimplemented.
   *
   * @param name of the camera, which maps to its network table section
   * @param tags don't remember
   * @param strat because photonvision uses its own kalman filter, and this is dumb
   * @param robotToCamera transform 3d, should really be a consumer
   */
  public PhotonVisionCamera(
      String name, AprilTagFieldLayout tags, PoseStrategy strat, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(name);
  }
  
  /**
   * Calculates the translational deviations. This is important for Kalman filters.
   *
   * @param distance distance to the average tag
   * @param tagCount number of tags used to make the pose
   * @return Deviations in the {x, y, yaw} coordinates
   */
  protected double[] calculateTranslationalDeviations(double distance, double tagCount) {
    // Custom Standard Deviations
    var stddev = Math.pow(distance * m_specs.distance_scalar, m_specs.error_exponent);
    // commented out for now until we know what's what
    // stddev /= tagCount * VisionConstants.L3G.TagCountErrorScalar;
    // stddev *= (1 + m_robotSpeedSupplier.get() * VisionConstants.LL3G.RobotSpeedErrorScalar);
    return new double[] {
      m_specs.error_multiplier * stddev + m_specs.minimum_error,
      m_specs.error_multiplier * stddev + m_specs.minimum_error,
      1
    };
  }

  @Override
  protected double calculateConfidence(
      Pose3d pose, int tagCount, double distance, double deviation) {
    var rotationSpeed = Math.abs(m_robotRotationSpeedSupplier.get());
    var isMovingTooFast = m_specs.max_speed_acceptable < m_robotSpeedSupplier.get();
    var isRotatingTooFast = m_specs.max_rotation_acceptable < rotationSpeed;
    var isTooFar = m_specs.max_distance_acceptable < distance;
    if (isTooFar || isMovingTooFast || isRotatingTooFast) {
      return 0;
    }
    return 1.0;
  }

  @Override
  protected VisionData processMeasuredData(int idx) {
    if (m_poseData.timestamps.length <= idx) {
      return null;
    }

    var conf = calculateConfidence(
      m_poseData.pose[idx],
      m_poseData.tagCount[idx],
      m_poseData.averageTagDistance[idx],
      m_poseData.deviations[0]);

    return new VisionData(
      m_poseData.pose[idx],
      m_poseData.timestamps[idx],
      new double[] {m_poseData.deviations[idx], m_poseData.deviations[idx], 1},
      conf);
  }

  @Override
  protected void LoadNTQueueToVisionData() {
    var poses = m_camera.getAllUnreadResults();
    m_poseData.resize(poses.size());
    for (int idx = 0; idx < poses.size(); idx++) {
      PhotonPipelineResult pose = poses.get(idx);
      // Already factors in pipeline latency
      double timestamp = pose.getTimestampSeconds();

      Transform3d p = pose.getMultiTagResult().get().estimatedPose.best;
      int num_tags = pose.getMultiTagResult().get().fiducialIDsUsed.size();
      double distance = 0.0;
      for (var t : pose.getTargets()) {
        distance += t.getBestCameraToTarget().getTranslation().getNorm();
      }
      distance /= num_tags;
      var deviations = calculateTranslationalDeviations(distance, (double)num_tags);

      m_poseData.timestamps[idx] = timestamp;
      m_poseData.pose[idx] = new Pose3d().plus(p);
      m_poseData.averageTagDistance[idx] = distance;
      m_poseData.deviations[idx] = deviations[0];
      m_poseData.tagCount[idx] = num_tags;
    }
  }

  @Override
  public boolean hasTarget() {
    return m_camera.getLatestResult().hasTargets();
  }

  @Override
  public double getTargetAzimuth(boolean cameraRelative) {
    return m_camera.getLatestResult().getBestTarget().getYaw();
  }

  @Override
  public double getTargetElevation(boolean cameraRelative) {
    return m_camera.getLatestResult().getBestTarget().getPitch();
  }

  @Override
  public int getPriorityID() {
    // PhotonLib doesn't support Priority IDs
    return 0;
  }

  @Override
  public void setPriorityID(int id) {
    // PhotonLib doesn't support Priority IDs
    return;
  }

  @Override
  public void resetPriorityID() {
    // PhotonLib doesn't support Priority IDs
    return;
  }

  @Override
  public void setMode(CameraMode mode) {
    m_mode = mode;
  }

  @Override
  public void setModeToDefault() {
    setMode(CameraMode.LOCALIZATION);
  }

  @Override
  public CameraMode getCurrentMode() {
    return m_mode;
  }

  @Override
  public void updateCropFromRobotpose(
      Pose3d robotPose, double horizontal_margin, double vertical_margin) {
    // PhotonLib doesn't support dynamic crop
    return;
  }

  @Override
  public void updateCropFromSpan(double l, double r, double t, double b) {
    // PhotonLib doesn't support dynamic crop
    return;
  }

  @Override
  public void resetCrop() {
    // PhotonLib doesn't support dynamic crop
    return;
  }
}
