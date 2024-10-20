package com.chaos131.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A data structure containing the calculated pose, timestamp, and deviation data
 */
@AutoLog
public class VisionData {
    /** 3d Pose, we store 3d because we can always convert down to 2d later */
    public Pose3d m_pose;
    /** Timestamp in seconds of when the image was taken, not the received pose */
    public double m_time;
    /** Deviation data to calculate confidence */
    public double m_deviation[];
    /** Confidence value of the pose */
    public double m_confidence;

    /**
     * @param pose calculated 3d pose
     * @param timestamp seconds
     * @param deviation for confidence
     * @param confidence for confidence
     */
    public void setData(Pose3d pose, double timestamp, double deviation[], double confidence) {
        m_pose = pose;
        m_time = timestamp;
        m_deviation = deviation;
        m_confidence = confidence;
    }

    /**
     * @return 2d form of the stored pose
     */
    public Pose2d getPose2d() {
        return m_pose.toPose2d();
    }

    /**
     * @return the original calculated pose
     */
    public Pose3d getPose3d() {
        return m_pose;
    }

    /**
     * @return the stored time
     */
    public double getTimestampSeconds() {
        return m_time;
    }

    /**
     * @return the deviation
     */
    public Matrix<N3,N1> getDeviationMatrix() {
        return VecBuilder.fill(m_deviation[0], m_deviation[1], m_deviation[2]);
    }

    /**
     * @return the deviation
     */
    public double[] getDeviation() {
        return m_deviation;
    }

    /**
     * @return the confidence of the pose
     */
    public double getConfidence() {
        return m_confidence;
    }
}
