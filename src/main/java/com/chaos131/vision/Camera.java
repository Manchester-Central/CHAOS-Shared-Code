package com.chaos131.vision;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract Class that defines what an FRC camera must implement to be useful.
 * 
 * This class is the gateway to robot localization, piece tracking, navigation, and more.
 */
public abstract class Camera extends SubsystemBase {
    /** Epsilon values exist to compare floating point values and see if something is "close enough" */
    protected final double EPSILON = 1e-8;
    /** Name of the Camera, typically used for NetworkTable access */
    protected String m_name;
    /** Specs for the camera, things like field of view, image sizes, and confidence factors */
    protected CameraSpecs m_specs;
    /** Flag to turn on and off a specific camera for localization updates on the fly */
    protected boolean m_useForOdometry;
    /** Timer to track how long until an update occured */
    private final Timer m_disconnectedTimer = new Timer();

    /** Supplies data while in simulation mode */
    protected Supplier<Pose2d> m_simPoseSupplier;
    /** Function that processes the VisionData captured, typically just a call to m_swerveDrive.addVisionMeasurement() */
    protected Consumer<VisionData> m_poseUpdator;
    /** Calculates the camera offset, useful for cameras attached to moving parts */
    protected Supplier<Pose3d> m_offset;
    /** Supplies the robot's speed based on wheel and odometry feedback */
    protected Supplier<Double> m_robotSpeedSupplier;
    /** Supplies the robot's rotation rate based on wheel and odometry feedback */
    protected Supplier<Double> m_robotRotationSpeedSupplier;

    /** NetworkTable name that corresponds with this Camera */
    protected NetworkTable m_visionTable;
    /** NetworkTable entry that supplies the robot's pose data */
    protected NetworkTableEntry m_botpose;
    /** Subscriber that loads in poses */
    private DoubleArraySubscriber m_observationSubscriber;
    /** NetworkTable entry that defines what state or processing pipeline the camera is currently in */
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
    /** If the target has a specific ID, it would be found in this network table entry */
    protected NetworkTableEntry m_targetID;
    /** If there's a priority id, it would be found in this network table entry */
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


    /*******************
     * Initializations *
     *******************/


    /**
     * Helps construct a Camera, should always be called in the child class constructors
     * @param name of the network table name, for instance "limelight-front"
     */
    public Camera(String name) {
        m_name = name;
        m_visionTable = NetworkTableInstance.getDefault().getTable(m_name);
        m_disconnectedTimer.start();
    }

    /**
     * Initialization function, sets the camera specs used for pose and targetting functions
     * @param specs the data to store
     * @return itself
     */
    protected Camera setSpecs(CameraSpecs specs) {
        m_specs = specs;
        return this;
    }
    /**
     * Initialization function, sets the name of the pipeline that carries pose info
     * @param name of the pose topic
     * @return itself
     */
    protected Camera setPosePipeline(String name) {
        m_botpose = m_visionTable.getEntry(name);
        m_observationSubscriber = 
            m_visionTable.getDoubleArrayTopic(name).subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true));
        return this;
    }
    /**
     * Initialization function, sets the name of the pipeline that carries target info
     * @param name of the target elevation topic
     * @return itself
     */
    protected Camera setTargetElevationPipeline(String name) {
        m_targetElevation = m_visionTable.getEntry(name);
        return this;
    }
    /**
     * Initialization function, sets the name of the pipeline that carries target info
     * @param name of the target azimuth topic
     * @return itself
     */
    protected Camera setTargetAzimuthPipeline(String name) {
        m_targetAzimuth = m_visionTable.getEntry(name);
        return this;
    }
    /**
     * Initialization function, sets the name of the pipeline that carries target info
     * @param name of the current camera pipeline topic
     * @return itself
     */
    protected Camera setGetCameraPipeline(String name) {
        m_pipelineID = m_visionTable.getEntry(name);
        return this;
    }
    /**
     * Initialization function, sets the name of the pipeline that carries priority target info
     * @param name of the priority id pipeline topic
     * @return itself
     */
    protected Camera setPriorityIDPipeline(String name) {
        m_priorityid = m_visionTable.getEntry(name);
        return this;
    }
    /**
     * Initialization function, sets the name of the pipeline that carries targeting info
     * @param name of the current target id pipeline topic
     * @return itself
     */
    protected Camera setTargetIDPipeline(String name) {
        m_targetID = m_visionTable.getEntry(name);
        return this;
    }
    /**
     * Will offset the pose when a valid pose is found in recordMeasuredData()
     * @param offsetHandler the function that defines the offset
     * @return itself
     */
    public Camera setOffsetHandler(Supplier<Pose3d> offsetHandler) {
        m_offset = offsetHandler;
        return this;
    }
    /**
     * Will send the VisionData to the SwerveDrive system if defined
     * @param poseConsumer the updater function
     * @return itself
     */
    public Camera setPoseUpdator(Consumer<VisionData> poseConsumer) {
        m_poseUpdator = poseConsumer;
        return this;
    }
    /**
     * Loopback function while in simulation mode to regurgitate the pose back to other systems
     * @param poseSupplier the pose function
     * @return itself
     */
    public Camera setSimPoseSupplier(Supplier<Pose2d> poseSupplier) {
        m_simPoseSupplier = poseSupplier;
        return this;
    }
    /**
     * Informs the Camera what the speed of the robot currently is.
     * <p>This is helpful because there is already a Kalman Filter in the SwerveDrive class that denoises data,
     * otherwise we would need a second kalman filter within the camera class.
     * @param speedSupplier the speed function
     * @return itself
     */
    public Camera setRobotSpeedSupplier(Supplier<Double> speedSupplier) {
        m_robotSpeedSupplier = speedSupplier;
        return this;
    }
    /**
     * Informs the Camera what the rotation rate of the robot currently is.
     * <p>This is helpful because there is already a Kalman Filter in the SwerveDrive class that denoises data,
     * otherwise we would need a second kalman filter within the camera class.
     * @param rotationSupplier the speed function
     * @return itself
     */
    public Camera setRobotRotationSupplier(Supplier<Double> rotationSupplier) {
        m_robotRotationSpeedSupplier = rotationSupplier;
        return this;
    }


    /************************
     * Pose Updator Methods *
     ************************/


    /**
     * Calculates the confidence of the pose based on several factors
     * @param pose the calculated pose
     * @param tagCount number of April tags in sight
     * @param distance average distance of all of the april tags
     * @param deviation a standard deviation of the results for the calculations (homebrew or from the device)
     * @return a value in the range of [0,1] (higher is better)
     */
    protected abstract double calculateConfidence(Pose3d pose, int tagCount, double distance, double deviation);

    /**
     * Processing Support Function that takes a network tables update, and converts it into a vision data
     * @param timestamp of the message in microseconds
     * @param serverTime of the message in microseconds
     * @param value the double array of values
     * @return vision data structure containing calculated poses
     */
    protected abstract VisionData processMeasuredData(long timestamp, long serverTime, double[] value);

    /**
     * Reads values off network tables, and hands off the queue of updates to another support function
     * @param queue array of TimestampedDouble's
     */
    protected void processUpdateQueue(TimestampedDoubleArray[] queue) {
        if (!m_useForOdometry || m_poseUpdator == null) {
            return;
        }

        for (var idx = 0; idx < queue.length; idx++) {
            var data = processMeasuredData(queue[idx].timestamp,
                                            queue[idx].serverTime,
                                            queue[idx].value);
            if (data != null) {
                m_poseUpdator.accept(data);
            }
            Logger.recordOutput(m_name+"/PoseTimestamp", queue[idx].timestamp);
            Logger.recordOutput(m_name+"/RobotPoses", data.getPose2d());
            Logger.recordOutput(m_name+"/RobotPoses3d", data.getPose3d());
        }

        if (queue.length > 0) {
            m_disconnectedTimer.reset();
        }
    }

    /**
     * Loads in VisionData from the log in replay mode, and sends vision updates
     * to the pose estimator during real mode.
     */
    @Override
    public void periodic() {
        processUpdateQueue(m_observationSubscriber.readQueue());
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


    /********************
     * States and Modes *
     ********************/


    /**
     * Run this to procedurally calculate crop regions on the fly
     * @param robotPose converts the given robot pose into a camera pose with the offset function (m_offset)
     */
    public abstract void updateCropFromRobotpose(Pose3d robotPose);
    /**
     * Sets the crop based on a tracked target span or coverage
     */
    public abstract void updateCropFromSpan();
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
