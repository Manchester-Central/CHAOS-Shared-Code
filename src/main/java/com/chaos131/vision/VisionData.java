package com.chaos131.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionData {
    private Pose3d m_pose;
    private double m_time;

    private Matrix <N3,N1> m_deviation;

    VisionData(Pose3d pose, double timestamp, Matrix <N3,N1>deviation) {
        m_pose = pose;
        m_time = timestamp;
        m_deviation = deviation;
    }

    public Pose2d getPose2d() {
        return m_pose.toPose2d();
    }

    public Pose3d getPose3d() {
        return m_pose;
    }

    public double getTimestampSeconds() {
        return m_time;
    }

    public Matrix <N3,N1> getDeviation() {
        return m_deviation; 
    }
}
