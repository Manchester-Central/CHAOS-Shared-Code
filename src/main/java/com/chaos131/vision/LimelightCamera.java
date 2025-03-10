package com.chaos131.vision;

import com.chaos131.util.Quad;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Implements a Camera behavior for the This is up to date for Limelight OS 2024.10.2 (10/28/24) */
public class LimelightCamera extends Camera {
  /** Limelight versions can help the implementation navigate features and calibration */
  public enum LimelightVersion {
    /** Limelight2 */
    LL2,
    /** Limelight3 */
    LL3,
    /** Limelight3G (the global shutter / greyscale one) */
    LL3G
  }

  /** Version of the limelight instance */
  protected LimelightVersion m_limeLightVersion;

  /**
   * Represents which mode the robot is in.
   *
   * <p>APRIL_TAGS - The pipeline
   *
   * <p>PIECE_TRACKING - The pipeline used for finding notes on the field. This is typically for
   * intake cameras, which may not be the forward camera.
   */
  public enum Mode {
    /** Focus on localization, and processing April Tags */
    APRIL_TAGS(0),
    /** Focus on tracking one or many game pieces */
    PIECE_TRACKING(1),
    /** Focus on a primary Blue side objective */
    BLUE_PRIMARY(2),
    /** Focus on a primary Red side objective */
    RED_PRIMARY(3),
    /** Focus on analyzing the environment */
    MAPPING(4);

    /** The pipeline ID for these features on the Limelight itself */
    public final Integer pipelineId;

    /**
     * @param pipelineId ID for the mode
     */
    private Mode(Integer pipelineId) {
      this.pipelineId = pipelineId;
    }
  }

  /** NetworkTable name that corresponds with this Camera */
  protected NetworkTable m_visionTable;

  /** NetworkTable entry that supplies the robot's pose data */
  protected NetworkTableEntry m_botpose;

  /** NetworkTable entry for MegaTag2 */
  protected NetworkTableEntry m_botposeMT2;

  /** A data structure used by AdvantageKit to record and replay Pose data */
  protected NetworkPoseDataAutoLogged m_poseDataMT2 = new NetworkPoseDataAutoLogged();

  /** Distance in meters before swapping from MT1 to MT2 */
  protected double m_megatag2Threshold;

  /** Subscriber that loads in poses */
  protected DoubleArraySubscriber m_observationSubscriber;

  /**
   * NetworkTable entry that defines what state or processing pipeline the camera is currently in
   */
  protected NetworkTableEntry m_pipelineID;

  /**
   * NetworkTable entry that defines the azimuth (left-right) of the current tracked target
   *
   * <p>This is typically in camera space, ranging from [-1,1]
   */
  protected NetworkTableEntry m_targetAzimuth;

  /**
   * NetworkTable entry that defines the elevation (bottom-top) of the current tracked target
   *
   * <p>This is typically in camera space, ranging from [-1,1]
   */
  protected NetworkTableEntry m_targetElevation;

  /** If the target has a specific ID, it would be found in this network table entry */
  protected NetworkTableEntry m_hasTarget;

  /** If there's a priority id, it would be found in this network table entry */
  protected NetworkTableEntry m_priorityid;

  /** Network Table Indices for Limelight OS 2024.4 */

  /** NT Entry ID - Pose X */
  protected final int idxX = 0;

  /** NT Entry ID - Pose Y */
  protected final int idxY = 1;

  /** NT Entry ID - Pose Z */
  protected final int idxZ = 2;

  /** NT Entry ID - Pose Roll */
  protected final int idxRoll = 3;

  /** NT Entry ID - Pose Pitch */
  protected final int idxPitch = 4;

  /** NT Entry ID - Pose Yaw */
  protected final int idxYaw = 5;

  /** NT Entry ID - Pipeline Latency */
  protected final int idxLatency = 6;

  /** NT Entry ID - Tags Seen */
  protected final int idxTagCount = 7;

  /** NT Entry ID - Tag Span */
  protected final int idxTagSpan = 8;

  /** NT Entry ID - Tag Distance Average */
  protected final int idxTagDistance = 9;

  /** NT Entry ID - Tag Area */
  protected final int idxTagArea = 10;

  /**
   * Constructs a limelight camera.
   *
   * @param name of the limelight as seen on network tables
   * @param limelightVersion for pulling calibration values
   * @param specs of the camera
   * @param poseSupplier supplies the current pose to the limelight
   * @param poseConsumer sends pose updates to another system, typically pose estimator in the
   *     swerve module
   * @param robotSpeedSupplier supplies the speed of the robot at that moment
   * @param robotRotationSpeedSupplier supplies the rotation rate of the robot at that moment
   */
  public LimelightCamera(
      String name,
      LimelightVersion limelightVersion,
      CameraSpecs specs,
      Supplier<Pose2d> poseSupplier,
      Consumer<VisionData> poseConsumer,
      Supplier<Double> robotSpeedSupplier,
      Supplier<Double> robotRotationSpeedSupplier) {
    super(name, specs);
    m_limeLightVersion = limelightVersion;
    m_visionTable = NetworkTableInstance.getDefault().getTable(m_name);
    m_botpose = m_visionTable.getEntry("botpose_wpiblue");
    m_botposeMT2 = m_visionTable.getEntry("botpose_orb_wpiblue");
    m_megatag2Threshold = 3.0; // Hard coded for now
    m_pipelineID = m_visionTable.getEntry("getpipe");
    m_targetAzimuth = m_visionTable.getEntry("tx");
    m_targetElevation = m_visionTable.getEntry("ty");
    m_hasTarget = m_visionTable.getEntry("tv");
    m_priorityid = m_visionTable.getEntry("priorityid");
    setSimPoseSupplier(poseSupplier);
    setPoseUpdator(poseConsumer);
    setRobotSpeedSupplier(robotRotationSpeedSupplier);
    setRobotRotationSupplier(robotRotationSpeedSupplier);
  }

  /**
   * Calculates the translational deviations. This is important for Kalman filters.
   *
   * @param distance distance to the average tag
   * @param tagCount number of tags used to make the pose
   * @return Deviations in the {x, y, yaw} coordinates
   */
  protected double[] calculateTranslationalDeviations(double distance, double tagCount) {
    // MegaTag Standard Deviations
    // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    // double[] devs = m_visionTable.getEntry("stddevs").getDoubleArray(new double[]{});
    // return new double[] {devs[0], devs[1], devs[5]};

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
  public double calculateConfidence(Pose3d pose, int tagCount, double distance, double deviation) {
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
  public boolean hasTarget() {
    return m_hasTarget.getInteger(0) != 0;
  }

  @Override
  public double getTargetAzimuth(boolean cameraRelative) {
    double cam_az = m_targetAzimuth.getDouble(Double.NaN);
    if (!cameraRelative) {
      if (m_offset != null) {
        Pose3d offset = m_offset.get();
        // Yaw is Z in WPILib
        cam_az -= offset.getRotation().getZ();
      }
    }
    return cam_az;
  }

  @Override
  public double getTargetElevation(boolean cameraRelative) {
    double cam_el = m_targetElevation.getDouble(Double.NaN);
    if (!cameraRelative) {
      if (m_offset != null) {
        Pose3d offset = m_offset.get();
        // Pitch is Y in WPILib
        cam_el -= offset.getRotation().getY();
      }
    }
    return cam_el;
  }

  @Override
  public int getPriorityID() {
    return (int) m_priorityid.getInteger(-1);
  }

  @Override
  public void setPriorityID(int id) {
    m_priorityid.setNumber(id);
  }

  @Override
  public void resetPriorityID() {
    m_priorityid.setNumber(-1);
  }

  @Override
  public void updateCropFromRobotpose(
      Pose3d robotPose, double horizontal_margin, double vertical_margin) {
    var visibletags =
        CameraTransforms.CalculateVisibleCoordinates(
            robotPose, (Quad[]) m_localizationTags.toArray(), EPSILON, EPSILON);
    // returns in the order of [minX, minY, maxX, maxY]
    var bounds = CameraTransforms.FindBounds(visibletags, horizontal_margin, vertical_margin);

    if (bounds == null || m_mode == CameraMode.PIECE_TRACKING) {
      m_visionTable.getEntry("crop").setDoubleArray(new double[] {-1, 1, -1, 1});
    } else {
      /**
       * Re-ordered because: 0 - Min or Max X value [-1, 1] 1 - Min or Max X value [-1, 1] 2 - Min
       * or Max Y value [-1, 1] 3 - Min or Max Y value [-1, 1]
       */
      m_visionTable
          .getEntry("crop")
          .setDoubleArray(new double[] {bounds[0], bounds[2], bounds[1], bounds[3]});
    }
  }

  @Override
  public void updateCropFromSpan(double l, double r, double t, double b) {
    m_visionTable.getEntry("crop").setDoubleArray(new double[] {l, r, b, t});
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
  public void resetCrop() {
    updateCropFromSpan(-1, 1, 1, -1);
  }

  @Override
  public VisionData processMeasuredData(int idx) {
    if (m_poseData.timestamps.length <= idx || m_poseDataMT2.timestamps.length <= idx) {
      // We have a size mismatch between MT1 and MT2, so the poses are desynched.
      return null;
    }

    // MT1
    var conf =
        calculateConfidence(
            m_poseData.pose[idx],
            m_poseData.tagCount[idx],
            m_poseData.averageTagDistance[idx],
            m_poseData.deviations[0]);
    var conf2 =
        calculateConfidence(
            m_poseDataMT2.pose[idx],
            m_poseDataMT2.tagCount[idx],
            m_poseDataMT2.averageTagDistance[idx],
            m_poseDataMT2.deviations[0]);

    if (m_poseData.averageTagDistance[idx] < m_megatag2Threshold) {
      return new VisionData(
          m_poseData.pose[idx],
          m_poseData.timestamps[idx],
          new double[] {m_poseData.deviations[idx], m_poseData.deviations[idx], 1},
          conf, m_name);
    } else {
      return new VisionData(
          m_poseDataMT2.pose[idx],
          m_poseDataMT2.timestamps[idx],
          new double[] {m_poseDataMT2.deviations[idx], m_poseDataMT2.deviations[idx], 1},
          conf2, m_name);
    }
  }

  @Override
  protected void LoadNTQueueToVisionData() {
    /** TODO: Serious race condition concern here! I can't simply fix this, Limelight needs to. */
    NetworkTableValue[] mt1_poses = m_botpose.readQueue();
    NetworkTableValue[] mt2_poses = m_botposeMT2.readQueue();

    // Parse MegaTag1 Info
    m_poseData.resize(mt1_poses.length);
    for (int idx = 0; idx < mt1_poses.length; idx++) {
      long timestamp = mt1_poses[idx].getServerTime();
      var data = mt1_poses[idx].getDoubleArray();

      double timestampSeconds = timestamp / 1000000.0 - data[idxLatency] / 1000.0;

      if (data == null || data[idxX] < EPSILON) {
        continue;
      }

      var posePosition = new Translation3d(data[idxX], data[idxY], data[idxZ]);

      var poseRotation =
          new Rotation3d(
              data[idxRoll] * Math.PI / 180,
              data[idxPitch] * Math.PI / 180,
              data[idxYaw] * Math.PI / 180);

      var visionPose = new Pose3d(posePosition, poseRotation);

      if (m_offset != null) {
        var cameraOffset = m_offset.get();
        cameraOffset = cameraOffset.rotateBy(new Rotation3d(0, 0, data[idxYaw] * Math.PI / 180));
        cameraOffset.getRotation().getZ();
        visionPose =
            new Pose3d(
                new Translation3d(
                    visionPose.getX() - cameraOffset.getX(),
                    visionPose.getY() - cameraOffset.getY(),
                    visionPose.getZ() - cameraOffset.getZ()),
                new Rotation3d(
                    0, // poseRotation.getX() - cameraOffset.getRotation().getX(),
                    0, // poseRotation.getY() - cameraOffset.getRotation().getY(),
                    poseRotation.getZ()) // - cameraOffset.getRotation().getZ())
                );
      }

      double[] deviation =
          calculateTranslationalDeviations(data[idxTagDistance], data[idxTagCount]);

      m_poseData.averageTagDistance[idx] = data[idxTagDistance];
      m_poseData.deviations[idx] = deviation[0];
      m_poseData.pose[idx] = visionPose;
      m_poseData.tagCount[idx] = (int) data[idxTagCount];
      m_poseData.timestamps[idx] = timestampSeconds;
    } // End MT1

    // Parse MegaTag2 Info
    m_poseDataMT2.resize(mt2_poses.length);
    for (int idx = 0; idx < mt2_poses.length; idx++) {
      long timestamp = mt2_poses[idx].getServerTime();
      var data = mt2_poses[idx].getDoubleArray();

      double timestampSeconds = timestamp / 1000000.0 - data[idxLatency] / 1000.0;

      if (data == null || data[idxX] < EPSILON) {
        continue;
      }

      var posePosition = new Translation3d(data[idxX], data[idxY], data[idxZ]);

      var poseRotation =
          new Rotation3d(
              data[idxRoll] * Math.PI / 180,
              data[idxPitch] * Math.PI / 180,
              data[idxYaw] * Math.PI / 180);

      var visionPose = new Pose3d(posePosition, poseRotation);

      if (m_offset != null) {
        var cameraOffset = m_offset.get();
        cameraOffset = cameraOffset.rotateBy(new Rotation3d(0, 0, data[idxYaw] * Math.PI / 180));
        cameraOffset.getRotation().getZ();
        visionPose =
            new Pose3d(
                new Translation3d(
                    visionPose.getX() - cameraOffset.getX(),
                    visionPose.getY() - cameraOffset.getY(),
                    visionPose.getZ() - cameraOffset.getZ()),
                new Rotation3d(
                    0, // poseRotation.getX() - cameraOffset.getRotation().getX(),
                    0, // poseRotation.getY() - cameraOffset.getRotation().getY(),
                    poseRotation.getZ()) // - cameraOffset.getRotation().getZ())
                );
      }

      double[] deviation =
          calculateTranslationalDeviations(data[idxTagDistance], data[idxTagCount]);

      m_poseDataMT2.averageTagDistance[idx] = data[idxTagDistance];
      m_poseDataMT2.deviations[idx] = deviation[0];
      m_poseDataMT2.pose[idx] = visionPose;
      m_poseDataMT2.tagCount[idx] = (int) data[idxTagCount];
      m_poseDataMT2.timestamps[idx] = timestampSeconds;
    } // End MT2
  }
}
