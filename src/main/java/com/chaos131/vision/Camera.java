package com.chaos131.vision;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Abstract Class that defines what an FRC camera must implement to be useful.
 * 
 * This class is the gateway to robot localization, piece tracking, navigation, and more.
 */
public abstract class Camera {
    /**
     * Epsilon values exist to compare floating point values and see if something is "close enough"
     */
    protected final double EPSILON = 1e-8;
    /**
     * Name of the Camera, typically used for NetworkTable access
     */
    protected String m_name;
    /**
     * Specs for the camera, things like field of view, image sizes, and confidence factors
     */
    protected CameraSpecs m_specs;
    /**
     * Flag to turn on and off a specific camera for localization updates on the fly
     */
    protected boolean m_useForOdometry;
    /**
     * Cache most recent data, note that there may not be any valid data
     */
    protected Optional<VisionData> m_mostRecentData; // caches the most recent data, including no-datas

    /**
     * Supplies data while in simulation mode
     */
    protected Supplier<Pose2d> m_simPoseSupplier;
    /**
     * Function that processes the VisionData captured, typically just a call to m_swerveDrive.addVisionMeasurement()
     */
    protected Consumer<VisionData> m_poseUpdator;
    /**
     * Calculates the camera offset, useful for cameras attached to moving parts
     */
    protected Supplier<Pose3d> m_offset;
    /**
     * Supplies the robot's speed based on wheel and odometry feedback
     */
    protected Supplier<Double> m_robotSpeedSupplier;
    /**
     * Supplies the robot's rotation rate based on wheel and odometry feedback
     */
    protected Supplier<Double> m_robotRotationSpeedSupplier;

    /**
     * NetworkTable name that corresponds with this Camera
     */
    protected NetworkTable m_visionTable;
    /**
     * NetworkTable entry that supplies the robot's pose data
     */
    protected NetworkTableEntry m_botpose;
    /**
     * NetworkTable entry that defines what state or processing pipeline the camera is currently in
     */
    protected NetworkTableEntry m_pipelineID;

    /**
     * NetworkTable entry that defines the azimuth (left-right) of the current tracked target
     * <p>This is typically in camera space, ranging from [-1,1]
     */
    protected NetworkTableEntry m_targetAzimuth;
    /**
     * NetworkTable entry that defines the elevation (bottom-top) of the current tracked target
     * <p>This is typically in camera space, ranging from [-1,1]
     */
    protected NetworkTableEntry m_targetElevation;
    /**
     * If the target has a specific ID, it would be found in this network table entry
     */
    protected NetworkTableEntry m_targetID;
    /**
     * If there's a priority id, it would be found in this network table entry
     */
    protected NetworkTableEntry m_priorityid;

    /**
     * There are only 3 primary modes for cameras in FRC: localization, tracking a game piece, mapping out the environment
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
     * Reads values off NetworkTables and generates a VisionData structure.
     * <p>If the PoseUpdator is defined, then it will attempt to update the pose
     */
    public abstract void recordMeasuredData();
    /**
     * Will offset the pose when a valid pose is found in recordMeasuredData()
     * @param offsetHandler the function that defines the offset
     */
    public abstract void setOffsetHandler(Supplier<Pose3d> offsetHandler);
    /**
     * Will send the VisionData to the SwerveDrive system if defined
     * @param poseConsumer the updater function
     */
    public abstract void setPoseUpdator(Consumer<VisionData> poseConsumer);
    /**
     * Loopback function while in simulation mode to regurgitate the pose back to other systems
     * @param poseSupplier the pose function
     */
    public abstract void setSimPoseSupplier(Supplier<Pose2d> poseSupplier);
    /**
     * Informs the Camera what the speed of the robot currently is.
     * <p>This is helpful because there is already a Kalman Filter in the SwerveDrive class that denoises data,
     * otherwise we would need a second kalman filter within the camera class.
     * @param speedSupplier the speed function
     */
    public abstract void setRobotSpeedSupplier(Supplier<Pose2d> speedSupplier);
    /**
     * Informs the Camera what the rotation rate of the robot currently is.
     * <p>This is helpful because there is already a Kalman Filter in the SwerveDrive class that denoises data,
     * otherwise we would need a second kalman filter within the camera class.
     * @param rotationSupplier the speed function
     */
    public abstract void setRobotRotationSupplier(Supplier<Pose2d> rotationSupplier);
    /**
     * @return the most recent floor pose stored in m_mostRecentData, returns null if empty
     */
    public abstract Pose2d getMostRecentPose();
    /**
     * Calculates the confidence of the pose based on several factors
     * @param pose the calculated pose
     * @param tagCount number of April tags in sight
     * @param distance average distance of all of the april tags
     * @param deviation a standard deviation of the results for the calculations (homebrew or from the device)
     * @return a value in the range of [0,1] (higher is better)
     */
    public abstract double calculateConfidence(Pose3d pose, int tagCount, double distance, double deviation);

    /**
     * @return true if there is a targetted april tag, game piece, or other resource
     */
    public abstract boolean hasTarget();
    /**
     * Gets the target azimuth (left and right)
     * @param cameraRelative true to calculate the angle of the target relative to the center of the camera, false to calculate it relative to the robot direction
     * @return angle in radians
     */
    public abstract double getTargetAzimuth(boolean cameraRelative);
    /**
     * Gets the target elevation (top and bottom)
     * @param cameraRelative true to calculate the angle of the target relative to the center of the camera, false to calculate it relative to the robot direction
     * @return angle in radians
     */
    public abstract double getTargetElevation(boolean cameraRelative);

    /**
     * @return the current priority ID, typically for april tag recognision
     */
    public abstract int getPriorityID();
    /**
     * @param id the priority id you want to track, -1 to reset (also see resetPriorityID)
     */
    public abstract void setPriorityID(int id);
    /**
     * Convenience function that calls setPriorityID(-1)
     */
    public abstract void resetPriorityID();

    /**
     * Run this to procedurally calculate crop regions on the fly
     * @param robotPose converts the given robot pose into a camera pose with the offset function (m_offset)
     */
    public abstract void updateCropFromRobotpose(Pose3d robotPose);
    /**
     * Sets the crop region back to the full image frame
     */
    public abstract void resetCrop();

    /**
     * @param mode that the camera should switch to
     */
    public abstract void setMode(CameraMode mode);
    /**
     * @param val true to enable odometry updates for this specific camera
     */
    public void setUseForOdometry(boolean val) { m_useForOdometry = val; }
    /**
     * Sets the camera to the default mode for its type
     */
    public abstract void setModeToDefault();
    /**
     * @return the current CameraMode, note that sub-modes are still converted to these CameraModes
     */
    public abstract CameraMode getCurrentMode();
    /**
     * @return true if the desired pipeline is the same as the reported pipeline
     */
    public abstract boolean isCorrectPipeline();
}
