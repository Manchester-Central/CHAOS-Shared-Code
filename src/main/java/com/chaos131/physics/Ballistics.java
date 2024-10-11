package com.chaos131.physics;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class Ballistics {
    public static Vector<N3> CalculateReleaseForUpwardsTarget(Vector<N3> release_position,
                                                              Vector<N3> target_position,
                                                              Vector<N3> robot_velocity,
                                                              double target_speed) {
        return null;
    }

    public static Vector<N3> CalculateReleaseForLandingSpot(Vector<N3> release_position,
                                                            Vector<N3> target_position,
                                                            Vector<N3> robot_velocity,
                                                            double desired_angle) {
        return null;
    }

    public static Vector<N3> IterateReleaseGradiantDescent(Vector<N3> current_release_vector,
                                                           Vector<N3> release_position,
                                                           Vector<N3> target_position,
                                                           Vector<N3> robot_velocity,
                                                           double target_speed) {
        return null;
    }
}
