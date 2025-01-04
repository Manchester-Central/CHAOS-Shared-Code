package com.chaos131.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.chaos131.util.Quad;

/**
 * Abstract Class that defines what an FRC camera must implement to be useful.
 *
 * <p>This class is the gateway to robot localization, piece tracking, navigation, and more.
 */
public abstract class Camera extends SubsystemBase {
  /**
   * A data structure recording and parsing information from network tables. This doesn't help a
   * camera recalculate pose data from a video feed, however we can re-analyze the pose data. Data
   * stored in here should have the number of timestamps match the number of data blocks (ie, pose
   * data). The individual pose data arrays can be arbitrarily sized to fit whatever Camera type is
   * being used (Limelight, Photon, Whatever).
   */
  @AutoLog
  class NetworkPoseData {
    /** Timestamps from NetworkTables, in microseconds */
    public double[] timestamps;
    /** Calculated Pose */
    public Pose3d[] pose;
    /** Number of tags observed in the creation of the pose */
    public int[] tagCount;
    /**  */
    public double[] deviations;
    /** Average distance of the tags used for the pose */
    public double[] averageTagDistance;

    public void reset() {
      timestamps = new double[]{};
      pose = new Pose3d[]{};
      tagCount = new int[]{};
      deviations = new double[]{};
      averageTagDistance = new double[]{};
    }

    public void resize(int s) {
      timestamps = new double[s];
      pose = new Pose3d[s];
      tagCount = new int[s];
      deviations = new double[s];
      averageTagDistance = new double[s];
    }
  }

  /**
   * A data structure recording targetting data. This is good for recording things like game piece
   * locations on the field. Note that the TargettingData is ambivalent about the number of
   * targets received, and so if many units are trying to be targetted then additional processing
   * should be done. Otherwise it will be difficult to distinguish corresponding AzEl locations.
   */
  @AutoLog
  class NetworkTargettingData {
    /** Timestamps from NetworkTables, in microseconds */
    public long[] timestamps = new long[] {};

    /** Values from the NetworkTables topic setup from setPosePipeline() */
    public double[] Az = new double[] {};
    /** Values from the NetworkTables topic setup from setPosePipeline() */
    public double[] El = new double[] {};

    public void reset() {
      timestamps = new long[] {};
      Az = new double[] {};
      El = new double[] {};
    }

    public void resize(int s) {
      timestamps = new long[s];
      Az = new double[s];
      El = new double[s];
    }
  }

  /**
   * Epsilon values exist to compare floating point values and see if something is "close enough"
   */
  protected final double EPSILON = 1e-8;

  /** Name of the Camera, typically used for NetworkTable access */
  protected String m_name;

  /** Specs for the camera, things like field of view, image sizes, and confidence factors */
  protected CameraSpecs m_specs;

  /** Flag to turn on and off a specific camera for localization updates on the fly */
  protected boolean m_useForOdometry;

  /** Timer to track how long until an update occured */
  protected final Timer m_disconnectedTimer = new Timer();

  /** Minimum time in seconds to be concerned about no tags being read */
  protected final double m_timerMinimum = 3.0;

  /** True if the robot is actively returning poses */
  protected boolean m_activeData = false;

  /** Supplies data while in simulation mode */
  protected Supplier<Pose2d> m_simPoseSupplier;

  /**
   * Function that changes the VisionData captured, typically just a call to
   * m_swerveDrive.addVisionMeasurement()
   */
  protected Consumer<VisionData> m_poseUpdator;

  /** Calculates the camera offset, useful for cameras attached to moving parts */
  protected Supplier<Pose3d> m_offset;

  /** Supplies the robot's speed based on wheel and odometry feedback */
  protected Supplier<Double> m_robotSpeedSupplier;

  /** Supplies the robot's rotation rate based on wheel and odometry feedback */
  protected Supplier<Double> m_robotRotationSpeedSupplier;

  /** A data structure used by AdvantageKit to record and replay Pose data */
  protected NetworkPoseDataAutoLogged m_poseData = new NetworkPoseDataAutoLogged();
  /** A data structure used by AdvantageKit to record and targetting data */
  protected NetworkTargettingDataAutoLogged m_targetData = new NetworkTargettingDataAutoLogged();

  /**
   * A list of April Tags we care about.
   * This is typically the list of all april tags on the field.
   */
  protected ArrayList<Quad> m_localizationTags = new ArrayList<Quad>();

  /**
   * There are only 3 core modes for cameras in FRC: localization, tracking a game piece, mapping
   * out the environment. A camera might have substates that break things like PIECE_TRACKING down
   * into specific modes for tracking different types of game pieces.
   */
  public enum CameraMode {
    /** Tells the camera to focus on positioning */
    LOCALIZATION,
    /** Tells the camera to focus on identifying game pieces and their locations */
    PIECE_TRACKING,
    /** Tells the camera to focus on mapping out the immediate environment */
    MAPPING
  };

  /**
   * A high level operating mode for the camera system. Individual cameras may or may not
   * implement specific sub-states or sub-modes. For instance, multiple piece tracking modes.
   */
  protected CameraMode m_mode;

  /*******************
   * Initializations *
   *******************/

  /** Empty constructor that should never be directly called */
  protected Camera() {}

  /**
   * Helps construct a Camera, should always be called in the child class constructors.
   *
   * @param name of the network table name, for instance "limelight-front"
   * @param specs specs for the camera, that may include physics and optical specs,
   * but could also include 
   */
  public Camera(String name, CameraSpecs specs) {
    m_name = name;
    m_specs = specs;
    m_useForOdometry = true;
    m_disconnectedTimer.start();

    m_poseData.reset();
    m_targetData.reset();
  }

  /**
   * Will offset the pose when a valid pose is found in recordMeasuredData()
   *
   * @param offsetHandler the function that defines the offset
   * @return itself
   */
  public Camera setOffsetHandler(Supplier<Pose3d> offsetHandler) {
    m_offset = offsetHandler;
    return this;
  }

  /**
   * Will send the VisionData to the SwerveDrive system if defined
   *
   * @param poseConsumer the updater function
   * @return itself
   */
  public Camera setPoseUpdator(Consumer<VisionData> poseConsumer) {
    m_poseUpdator = poseConsumer;
    return this;
  }

  /**
   * Loopback function while in simulation mode to regurgitate the pose back to other systems
   *
   * @param poseSupplier the pose function
   * @return itself
   */
  public Camera setSimPoseSupplier(Supplier<Pose2d> poseSupplier) {
    m_simPoseSupplier = poseSupplier;
    return this;
  }

  /**
   * Informs the Camera what the speed of the robot currently is.
   *
   * <p>This is helpful because there is already a Kalman Filter in the SwerveDrive class that
   * denoises data, otherwise we would need a second kalman filter within the camera class.
   *
   * @param speedSupplier the speed function
   * @return itself
   */
  public Camera setRobotSpeedSupplier(Supplier<Double> speedSupplier) {
    m_robotSpeedSupplier = speedSupplier;
    return this;
  }

  /**
   * Informs the Camera what the rotation rate of the robot currently is.
   *
   * <p>This is helpful because there is already a Kalman Filter in the SwerveDrive class that
   * denoises data, otherwise we would need a second kalman filter within the camera class.
   *
   * @param rotationSupplier the speed function
   * @return itself
   */
  public Camera setRobotRotationSupplier(Supplier<Double> rotationSupplier) {
    m_robotRotationSpeedSupplier = rotationSupplier;
    return this;
  }

  public Camera setTagsOfInterest(ArrayList<Quad> tags) {
    m_localizationTags = tags;
    return this;
  }

  /************************
   * Pose Updator Methods *
   ************************/

  /**
   * Calculates the confidence of the pose based on several factors
   *
   * @param pose the calculated pose
   * @param tagCount number of April tags in sight
   * @param distance average distance of all of the april tags
   * @param deviation a standard deviation of the results for the calculations (homebrew or from the
   *     device)
   * @return a value in the range of [0,1] (higher is better)
   */
  protected abstract double calculateConfidence(Pose3d pose, int tagCount, double distance, double deviation);

  /**
   * Processing Support Function that takes a network tables update, and converts it into a vision
   * data
   *
   * @param idx the index number of the pose data in m_visionData
   * @return vision data structure containing calculated poses
   */
  protected abstract VisionData processMeasuredData(int idx);

  /**
   * Reads values off network tables, and hands off the queue of updates to another support function
   */
  protected void processUpdateQueue() {
    if (!m_useForOdometry || m_poseUpdator == null) {
      // Don't have to do anything fancy, periodic() has flushed the message queue
      return;
    }

    for (var idx = 0; idx < m_poseData.timestamps.length; idx++) {
      VisionData data = processMeasuredData(idx);
      if (data != null) {
        m_poseUpdator.accept(data);
      }

      Logger.recordOutput(m_name + "/PoseTimestamp", data.getTimestampSeconds());
      Logger.recordOutput(m_name + "/RobotPose", data.getPose2d());
      Logger.recordOutput(m_name + "/RobotPose3d", data.getPose3d());
      Logger.recordOutput(m_name + "/Confidence", data.getConfidence());
      Logger.recordOutput(m_name + "/Deviation", data.getDeviation());
    }
  }

  /**
   * Reads data from the network table message queue(s), and loads it into the Pose and Targetting
   * structures used for AdvantageKit.
   */
  abstract protected void LoadNTQueueToVisionData();

  /**
   * Loads in VisionData from the log in replay mode, and sends vision updates to the pose estimator
   * during real mode. Probably shouldn't override this.
   */
  @Override
  public void periodic() {
    LoadNTQueueToVisionData();

    /**
     * This step will replace all that data (ie, no data) if we're in replay mode by reading values
     * from the log file. If we're a real robot, we instead log the data TO the file!
     */
    Logger.processInputs(m_name, m_poseData);

    /**
     * Now we do that thang with the all the data we received.
     */
    processUpdateQueue();

    /** If the timer has expired, so set the state to inactive... */
    if (m_poseData.timestamps.length > 0 || m_targetData.timestamps.length > 0) {
      m_disconnectedTimer.reset();
      m_activeData = true;
    } else {
      m_activeData = false;
    }
    Logger.recordOutput(m_name + "/ActiveData", m_activeData);
    Logger.recordOutput(
        m_name + "/DisconnectTimer",
        m_disconnectedTimer.get() < m_timerMinimum ? 0.0 : m_disconnectedTimer.get());
  }

  /***********************
   * Targeting Functions *
   ***********************/

  /**
   * @return true if there is a targetted april tag, game piece, or other resource
   */
  public abstract boolean hasTarget();

  /**
   * Gets the target azimuth (left and right)
   *
   * @param cameraRelative true to calculate the angle of the target relative to the center of the
   *     camera, false to calculate it relative to the robot direction
   * @return angle in radians
   */
  public abstract double getTargetAzimuth(boolean cameraRelative);

  /**
   * Gets the target elevation (top and bottom)
   *
   * @param cameraRelative true to calculate the angle of the target relative to the center of the
   *     camera, false to calculate it relative to the robot direction
   * @return angle in radians
   */
  public abstract double getTargetElevation(boolean cameraRelative);

  /**
   * @return the current priority ID, typically for april tag recognision
   */
  public abstract int getPriorityID();

  /**
   * @param id the priority id you want to track
   */
  public abstract void setPriorityID(int id);

  /** Convenience function that should call setTargetID(id) */
  public abstract void resetPriorityID();

  /********************
   * States and Modes *
   ********************/

  /**
   * Run this to procedurally calculate crop regions on the fly
   *
   * @param robotPose converts the given robot pose into a camera pose with the offset function
   *     (m_offset)
   */
  public abstract void updateCropFromRobotpose(
    Pose3d robotPose, double horizontal_margin, double vertical_margin);

  /** Sets the crop based on a tracked target span or coverage */
  public abstract void updateCropFromSpan(double l, double r, double t, double b);

  /** Sets the crop region back to the full image frame */
  public abstract void resetCrop();

  /**
   * Takes a high level camera operational mode. This may need to be overwritten and referenced back
   * with super().
   * 
   * @param mode that the camera should switch to
   */
  public abstract void setMode(CameraMode mode);

  /** Sets the camera to the default mode for its type */
  public abstract void setModeToDefault();

  /**
   * @return the current CameraMode, note that sub-modes are still converted to these CameraModes
   */
  public abstract CameraMode getCurrentMode();

  /**
   * @param val true to enable odometry updates for this specific camera
   */
  public void setUseForOdometry(boolean val) {
    m_useForOdometry = val;
  }
}
