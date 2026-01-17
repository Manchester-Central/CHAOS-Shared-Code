package com.chaos131.poses;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PivotedFieldPose extends FieldPose {
  protected PivotedFieldPose(Translation2d midpoint, Alliance defaultAlliance, String name, Pose3d defaultPose) {
    super(midpoint, defaultAlliance, name, defaultPose);
  }

  public Pose3d calculateSymmetry(Translation2d midpoint, Pose3d pose) {
    // Probably not necessary to raise the point, but do it just in case
    Translation3d rotationPoint = new Translation3d(midpoint.getX(), midpoint.getY(), pose.getTranslation().getZ());
    return pose.rotateAround(rotationPoint, new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180)));
  }
}
