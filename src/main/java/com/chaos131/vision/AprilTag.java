package com.chaos131.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N4;

/**
 * Defines an april tag in the world, and stores information about its id, location, size, and orientation on the field.
 * 
 * Really just used for vision systems.
 */
public class AprilTag /* Does this extend anything? */ {
	
    public final int id;// The ID number for the April Tag
    public final String family; // Describes the AprilTag Family, FRC 2024 uses 36h11

    public AprilTag(int tag_id, Matrix<N4, N4> transform, double sizeInMeters) {
        this(tag_id, "36h11", transform, sizeInMeters);
    }

    public AprilTag(int tag_id, String tag_family, Matrix<N4, N4> transform, double sizeInMeters) {
        id = tag_id;
        family = tag_family;
        // Implement what's needed below
    }
}