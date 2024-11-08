package com.chaos131.vision;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract Class that defines what an FRC camera must implement to be useful.
 * 
 * This class is the gateway to robot localization, piece tracking, navigation, and more.
 */
public abstract class Camera extends SubsystemBase {
    /**
     * A data structure recording and parsing information from network tables.
     * This doesn't help a camera recalculate pose data from a video feed,
     * however we can re-analyze the pose data. Data stored in here should have
     * the number of timestamps match the number of data blocks (ie, pose data).
     * The individual pose data arrays can be arbitrarily sized to fit whatever
     * Camera type is being used (Limelight, Photon, Whatever).
     */
    class NetworkPoseData implements LoggableInputs {
        /** Timestamps from NetworkTables, in microseconds */
        public long[] timestamps = new long[] {};
        /** Values from the NetworkTables topic setup from setPosePipeline() */
        public double[][] values = new double[][] {};

        @Override
        public void toLog(LogTable table) {
            table.put("Timestamps", timestamps);
            // Necessary to size the array in fromLog()
            table.put("FrameCount", values.length);
            for (int i = 0; i < values.length; i++) {
                table.put("Frame/" + i, values[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            timestamps = table.get("Timestamps", new long[] {0});
            int frameCount = table.get("FrameCount", 0);
            values = new double[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                values[i] = table.get("Frame/" + i, new double[] {});
            }
        }
    }

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
    /** True if the robot is actively returning poses */
    private boolean m_activeData = false;

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
    /** A data structure used by AdvantageKit to record and replay data */
    private NetworkPoseData m_visionData = new NetworkPoseData();
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
    protected NetworkTableEntry m_hasTarget;
    /** If there's a priority id, it would be found in this network table entry */
    protected NetworkTableEntry m_priorityid;

    /**
     * There are only 3 core modes for cameras in FRC: localization, tracking a game piece, mapping out the environment.
     * A camera might have substates that break things like PIECE_TRACKING down into specific modes for tracking different
     * types of game pieces.
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
    public Camera setSpecs(CameraSpecs specs) {
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
    protected Camera setHasTargetPipeline(String name) {
        m_hasTarget = m_visionTable.getEntry(name);
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
     * @param data the double array of values
     * @return vision data structure containing calculated poses
     */
    protected abstract VisionData processMeasuredData(long timestamp, double[] data);

    /** Reads values off network tables, and hands off the queue of updates to another support function */
    private void processUpdateQueue() {
        if (!m_useForOdometry || m_poseUpdator == null) {
            // Don't have to do anything fancy, periodic() has flushed the message queue
            return;
        }

        for (var idx = 0; idx < m_visionData.timestamps.length; idx++) {
            var data = processMeasuredData(m_visionData.timestamps[idx], m_visionData.values[idx]);
            if (data != null) {
                m_poseUpdator.accept(data);
            }

            Logger.recordOutput(m_name+"/PoseTimestamp", data.getTimestampSeconds());
            Logger.recordOutput(m_name+"/RobotPose", data.getPose2d());
            Logger.recordOutput(m_name+"/RobotPose3d", data.getPose3d());
            Logger.recordOutput(m_name+"/Confidence", data.getConfidence());
            Logger.recordOutput(m_name+"/Deviation", data.getDeviation());
        }
    }

    /**
     * Loads in VisionData from the log in replay mode, and sends vision updates
     * to the pose estimator during real mode. Probably shouldn't override this.
     */
    @Override
    public void periodic() {
        /**
         * This block does a bunch of work to read off the network tables
         */
        var queue = m_observationSubscriber.readQueue();
        m_visionData.timestamps = new long[queue.length];
        m_visionData.values = new double[queue.length][];
        // for each message, lets loop through and grab the timestamp then the data
        for (var idx = 0; idx < queue.length; idx++) {
            m_visionData.timestamps[idx] = queue[idx].timestamp;
            // now we need the data, 2d arrays are annoying!
            m_visionData.values[idx] = new double[queue[idx].value.length];
            for (var idx2 = 0; idx2 < queue[idx].value.length; idx2++) {
                m_visionData.timestamps[idx] = queue[idx].timestamp;
            }
        }

        /**
         * This step will replace all that data (ie, no data) if we're in replay mode 
         * by reading values from the log file. If we're a real robot, we instead log
         * the data TO the file!
         */ 
        Logger.processInputs(m_name, m_visionData);
        processUpdateQueue();

        /** If the timer has expired, so set the state to inactive... */
        if (m_visionData.timestamps.length > 0) {
            m_disconnectedTimer.reset();
            m_activeData = true;
        } else {
            m_activeData = false;
        }
        Logger.recordOutput(m_name+"/ActiveData", m_activeData);
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
