package com.chaos131.physics;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

/**
 * A utility class to calculate release vectors. Most functions are static and state-less. In
 * another language, like C++, we would probably toss this in a namespace instead.
 */
public class Ballistics {
  /**
   * UNIMPLEMENTED - For Students to complete!
   *
   * <p>Calculates a release vector when trying to launch a game piece up into the air. It solves
   * the equation ignoring air resistance, and uses the target_speed as the upwards speed at the
   * moment the game piece reaches the target position. All points must be in a consistent
   * coordinate frame. Either stick in field coordinates, or derive robot relative coordinates.
   *
   * <p>Note that because this ignores air resistance, it is not a full physics model, but it should
   * be accurate enough in most game scenarios if the mechanical team designed their system well.
   *
   * @param release_position the coordinate of the release point, not the robot pose
   * @param target_position the coordinate of the target point we want the ballistic to reach
   * @param robot_velocity the speed of the robot at that instant
   * @param target_speed the upwards speed of the game piece as it reaches the target_position
   * @return the final release vector
   */
  public static Vector<N3> CalculateReleaseForUpwardsTarget(
      Vector<N3> release_position,
      Vector<N3> target_position,
      Vector<N3> robot_velocity,
      double target_speed) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'CalculateReleaseForUpwardsTarget'");
  }

  /**
   * UNIMPLEMENTED - For Students to complete!
   *
   * <p>Calculates a release vector when trying to lob a game piece into a specific spot on the
   * ground. It solves the equation ignoring air resistance, and uses the desired angle to determine
   * the resulting velocity that will meet that landing spot.
   *
   * <p>All points must be in a consistent coordinate frame. Either stick in field coordinates, or
   * derive robot relative coordinates.
   *
   * <p>Note that because this ignores air resistance, it is not a full physics model, but it should
   * be accurate enough in most game scenarios if the mechanical team designed their system well.
   * Any resistance and resulting "falling short" will hopefully be compensated by any bouncing that
   * happens once it hits the ground.
   *
   * @param release_position the coordinate of the release point, not the robot pose
   * @param target_position the coordinate of the target point we want the ballistic to reach
   * @param robot_velocity the speed of the robot at that instant
   * @param desired_angle the angle you want the launcher to launch at
   * @return the final release vector
   */
  public static Vector<N3> CalculateReleaseForLandingSpot(
      Vector<N3> release_position,
      Vector<N3> target_position,
      Vector<N3> robot_velocity,
      double desired_angle) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'CalculateReleaseForLandingSpot'");
  }

  /**
   * UNIMPLEMENTED - For Students to complete!
   *
   * <p>Iterates the current release vector to move closer to the target state
   *
   * @param current_release_vector the current release vector to iterate off of
   * @param release_position the coordinate of the release point, not the robot pose
   * @param target_position the coordinate of the target point we want the ballistic to reach
   * @param robot_velocity the speed of the robot at that instant
   * @param target_speed speed you want the piece to move, opposite the direction of gravity
   * @param step_size scalar for the change in values for each iteration
   * @return the adjusted release vector
   */
  public static Vector<N3> IterateReleaseGradiantDescent(
      Vector<N3> current_release_vector,
      Vector<N3> release_position,
      Vector<N3> target_position,
      Vector<N3> robot_velocity,
      double target_speed,
      double step_size) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'IterateReleaseGradiantDescent'");
  }
}
