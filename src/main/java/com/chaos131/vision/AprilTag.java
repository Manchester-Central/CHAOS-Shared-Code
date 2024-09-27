package com.chaos131.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N4;

public class AprilTag /* Does this extend anything? */ {

    // The ID number for the April Tag
    public final int id = 1;

    // Describes the AprilTag Family, FRC 2024 uses 36h11
    public final String family = "";

    public AprilTag(int tag_id, Matrix<N4, N4> transform, double sizeInMeters) {
        this(tag_id, "36h11", transform, sizeInMeters);
    }

    public AprilTag(int tag_id, String tag_family, Matrix<N4, N4> transform, double sizeInMeters) {
        // Implement what's needed
    }
}