package com.chaos131.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** An unimplemented class to process PhotonVision Updates */
public class PhotonVisionCamera extends Camera {
  @SuppressWarnings("unused")
  private PhotonCamera m_camera;

  private PhotonPoseEstimator m_poseEstimator;

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
    m_poseEstimator = new PhotonPoseEstimator(tags, strat, robotToCamera);
  }

  @Override
  protected double calculateConfidence(
      Pose3d pose, int tagCount, double distance, double deviation) {
    return 1;
  }

  @Override
  protected VisionData processMeasuredData(int idx) {
    // List<PhotonPipelineResult> new_res = m_camera.getAllUnreadResults();
    Optional<EstimatedRobotPose> new_pose = m_poseEstimator.update(null);
    return new VisionData(
        new_pose.get().estimatedPose, new_pose.get().timestampSeconds, new double[] {0, 0, 0}, 1, m_name);
  }

  @Override
  public boolean hasTarget() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'hasTarget'");
  }

  @Override
  public double getTargetAzimuth(boolean cameraRelative) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTargetAzimuth'");
  }

  @Override
  public double getTargetElevation(boolean cameraRelative) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTargetElevation'");
  }

  @Override
  public int getPriorityID() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getPriorityID'");
  }

  @Override
  public void setPriorityID(int id) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPriorityID'");
  }

  @Override
  public void resetPriorityID() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetPriorityID'");
  }

  @Override
  public void resetCrop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'resetCrop'");
  }

  @Override
  public void setMode(CameraMode mode) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setMode'");
  }

  @Override
  public void setModeToDefault() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setModeToDefault'");
  }

  @Override
  public CameraMode getCurrentMode() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getCurrentMode'");
  }

  @Override
  protected void LoadNTQueueToVisionData() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'LoadNTQueueToVisionData'");
  }

  @Override
  public void updateCropFromRobotpose(
      Pose3d robotPose, double horizontal_margin, double vertical_margin) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateCropFromRobotpose'");
  }

  @Override
  public void updateCropFromSpan(double l, double r, double t, double b) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateCropFromSpan'");
  }
}
