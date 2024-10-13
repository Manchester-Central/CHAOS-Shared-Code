package com.chaos131.vision;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

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
    private LimelightVersion m_limeLightVersion;

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

    @Override
    public void recordMeasuredData() {
        var data = m_botpose.getValue().getDoubleArray();
        double timestampSeconds = Timer.getFPGATimestamp() - data[idxLatency] / 1000;
        if (data == null || data[idxX] < EPSILON) {
            m_mostRecentData = Optional.empty();
            return;
        }

        var poseRotation = new Rotation3d(    data[idxRoll] * Math.PI / 180, 
                                            data[idxPitch] * Math.PI / 180,
                                            data[idxYaw]  * Math.PI / 180);

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
        var trackXYZ = MatBuilder.fill(Nat.N3(), Nat.N1(), new double[]{ deviation, deviation, 1 });

        var conf = calculateConfidence(visionPose, (int)data[idxTagCount], data[idxTagDistance], deviation);
        if (conf < m_specs.confidence_requirement) {
            m_mostRecentData = Optional.empty();
            return;
        }

        m_mostRecentData = Optional.of(new VisionData(visionPose, timestampSeconds, trackXYZ));
        if (m_poseUpdator != null && m_useForOdometry) {
            m_poseUpdator.accept(m_mostRecentData.get());
        }
    }

    private double calculateTranslationalDeviations(double distance, double tagCount) {
        var stddev = Math.pow(distance*m_specs.distance_scalar, m_specs.error_exponent);
        // commented out for now until we know what's what
        // stddev /= tagCount * VisionConstants.L3G.TagCountErrorScalar;
        // stddev *= (1 + m_robotSpeedSupplier.get() * VisionConstants.LL3G.RobotSpeedErrorScalar);
        return m_specs.error_multiplier * stddev + m_specs.minimum_error;
    }

    @Override
    public void setOffsetHandler(Supplier<Pose3d> offsetHandler) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setOffsetHandler'");
    }

    @Override
    public void setPoseUpdator(Consumer<VisionData> poseConsumer) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPoseUpdator'");
    }

    @Override
    public void setSimPoseSupplier(Supplier<Pose2d> poseSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSimPoseSupplier'");
    }

    @Override
    public void setRobotSpeedSupplier(Supplier<Pose2d> speedSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRobotSpeedSupplier'");
    }

    @Override
    public void setRobotRotationSupplier(Supplier<Pose2d> rotationSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRobotRotationSupplier'");
    }

    @Override
    public Pose2d getMostRecentPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMostRecentPose'");
    }

    @Override
    public double calculateConfidence(Pose3d pose, int tagCount, double distance, double deviation) {
        var rotationSpeed = Math.abs(m_robotRotationSpeedSupplier.get());
        var isMovingTooFast = m_limeLightVersion == LimelightVersion.LL3G ? false : m_specs.max_speed_acceptable < m_robotSpeedSupplier.get();
        var isRotatingTooFast = rotationSpeed > m_specs.max_rotation_acceptable;
        var isTooFar =  m_limeLightVersion == LimelightVersion.LL3G ? distance > 4: m_specs.max_distance_acceptable < distance;
        if (isTooFar || isMovingTooFast || isRotatingTooFast) {
            return 0;
        }
        return 1.0;
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
    public void updateCropFromRobotpose(Pose3d robotPose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateCropFromRobotpose'");
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
}
