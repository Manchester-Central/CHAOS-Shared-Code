package com.chaos131.vision;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Implements a Camera behavior for the
 * This is up to date for Limelight OS 2024.4.0 (April 3rd, 2024)
 */
public class LimelightCamera extends Camera {
    /**
     * Limelight versions can help the implementation navigate features and calibration
     */
    public enum LimelightVersion {
        /** Limelight2 */
        LL2,
        /** Limelight3 */
        LL3,
        /** Limelight3G (the global shutter / greyscale one) */
        LL3G
    }

    /**
     * Version of the limelight instance
     */
    protected LimelightVersion m_limeLightVersion;

    /**
     * Represents which mode the robot is in.
     * 
     * <p>APRIL_TAGS - The pipeline 
     * <p>PIECE_TRACKING - The pipeline used for finding notes on the field.
     * This is typically for intake cameras, which may not be the forward camera.
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

    /**
     * Network Table Indices for Limelight OS 2024.4
     */

    /** NT Entry ID - Pose X */
    private final int idxX = 0;
    /** NT Entry ID - Pose Y */
    private final int idxY = 1;
    /** NT Entry ID - Pose Z */
    private final int idxZ = 2;
    /** NT Entry ID - Pose Roll */
    private final int idxRoll = 3;
    /** NT Entry ID - Pose Pitch */
    private final int idxPitch = 4;
    /** NT Entry ID - Pose Yaw */
    private final int idxYaw = 5;
    /** NT Entry ID - Pipeline Latency */
    private final int idxLatency = 6;
    /** NT Entry ID - Tags Seen */
    private final int idxTagCount = 7;
    /** NT Entry ID - Tag Span */
    private final int idxTagSpan = 8;
    /** NT Entry ID - Tag Distance Average */
    private final int idxTagDistance = 9;
    /** NT Entry ID - Tag Area */
    private final int idxTagArea = 10;

    /**
     * Constructs a limelight camera.
     * @param name of the limelight as seen on network tables
     * @param limelightVersion for pulling calibration values
     * @param specs of the camera
     * @param poseSupplier supplies the current pose to the limelight
     * @param poseConsumer sends pose updates to another system, typically pose estimator in the swerve module
     * @param robotSpeedSupplier supplies the speed of the robot at that moment
     * @param robotRotationSpeedSupplier supplies the rotation rate of the robot at that moment
     */
    public LimelightCamera(String name, LimelightVersion limelightVersion, CameraSpecs specs,
                            Supplier<Pose2d> poseSupplier, Consumer<VisionData> poseConsumer,
                            Supplier<Double> robotSpeedSupplier, Supplier<Double> robotRotationSpeedSupplier) {
        super(name);
        m_limeLightVersion = limelightVersion;
        setSpecs(specs);
        setPosePipeline("botpose_wpiblue");
        setGetCameraPipeline("getpipe");
        setTargetElevationPipeline("tx");
        setTargetAzimuthPipeline("ty");
        setSimPoseSupplier(poseSupplier);
        setPoseUpdator(poseConsumer);
        setRobotSpeedSupplier(robotRotationSpeedSupplier);
        setRobotRotationSupplier(robotRotationSpeedSupplier);
        setPriorityIDPipeline("priorityid");
        setHasTargetPipeline("tv");
    }

    private double calculateTranslationalDeviations(double distance, double tagCount) {
        var stddev = Math.pow(distance*m_specs.distance_scalar, m_specs.error_exponent);
        // commented out for now until we know what's what
        // stddev /= tagCount * VisionConstants.L3G.TagCountErrorScalar;
        // stddev *= (1 + m_robotSpeedSupplier.get() * VisionConstants.LL3G.RobotSpeedErrorScalar);
        return m_specs.error_multiplier * stddev + m_specs.minimum_error;
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
        if(!cameraRelative) {
            throw new UnsupportedOperationException("'cameraRelative: false' is not implemented!");
        }
        return m_targetAzimuth.getDouble(Double.NaN);
    }

    @Override
    public double getTargetElevation(boolean cameraRelative) {
        if(!cameraRelative) {
            throw new UnsupportedOperationException("'cameraRelative: false' is not implemented!");
        }
        return m_targetElevation.getDouble(Double.NaN);
    }

    @Override
    public int getPriorityID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPriorityID'");
    }

    @Override
    public void setPriorityID(int id) {
        m_priorityid.setInteger(id);
    }

    @Override
    public void resetPriorityID() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPriorityID'");
    }

    @Override
    public void updateCropFromRobotpose(Pose3d robotPose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateCropFromRobotpose'");
    }

    @Override
    public void updateCropFromSpan() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateCropFromSpan'");
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
    public boolean isCorrectPipeline() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isCorrectPipeline'");
    }

    @Override
    public void resetCrop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetCrop'");
    }

    @Override
    public VisionData processMeasuredData(long timestamp, double[] data) {
        double timestampSeconds = timestamp/1000000.0 - data[idxLatency] / 1000.0;
        if (data == null || data[idxX] < EPSILON) {
            return null;
        }

        var poseRotation = new Rotation3d(data[idxRoll]  * Math.PI / 180, 
                                          data[idxPitch] * Math.PI / 180,
                                          data[idxYaw]   * Math.PI / 180);

        var visionPose = new Pose3d(data[idxX], data[idxY], data[idxZ], poseRotation);

        if (m_offset != null) {
            var cameraOffset = m_offset.get();
            cameraOffset = cameraOffset.rotateBy(new Rotation3d(0, 0, data[idxYaw] * Math.PI / 180));
            visionPose = new Pose3d(
                new Translation3d(visionPose.getX() - cameraOffset.getX(),
                                visionPose.getY() - cameraOffset.getY(),
                                visionPose.getZ() - cameraOffset.getZ()),
                new Rotation3d(0,//poseRotation.getX() - cameraOffset.getRotation().getX(),
                                0,//poseRotation.getY() - cameraOffset.getRotation().getY(),
                                poseRotation.getZ())//- cameraOffset.getRotation().getZ())
            );
        }

        var deviation = calculateTranslationalDeviations(data[idxTagDistance], data[idxTagCount]);
        var trackXYZ = new double[]{deviation, deviation, 1 };
        var conf = calculateConfidence(visionPose, (int)data[idxTagCount], data[idxTagDistance], deviation);

        return new VisionData(visionPose, timestampSeconds, trackXYZ, conf);
    }
}
