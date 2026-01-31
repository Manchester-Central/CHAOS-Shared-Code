package com.chaos131.poses;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.IOException;

public class FieldPose2026 extends PivotedFieldPose {
  private static Translation2d Midpoint; // Must be initialized before any individual FieldPoses

  public static final Translation3d BlueHubCenter =
      new Translation3d(Inches.of(181.56), Inches.of(158.32), Inches.of(72));
  public static final Translation3d RedHubCenter =
      new Translation3d(Inches.of(468.56), Inches.of(158.32), Inches.of(72));
  public static final Translation3d BlueAllianceBallBox =
      new Translation3d(Inches.of(27.0 / 2), Inches.of(234.32), Inches.of(0));
  public static final Translation3d RedAllianceBallBox =
      new Translation3d(Inches.of(27.0 / 2), Inches.of(234.32), Inches.of(0));
  public static final Translation3d BlueClimbFarLeft =
      new Translation3d(Inches.of(41.8), Inches.of(123.96), Inches.of(0));
  public static final Translation3d BlueClimbLeft =
      new Translation3d(Inches.of(43.367), Inches.of(130.5), Inches.of(0));
  public static final Translation3d BlueClimbMiddle =
      new Translation3d(Inches.of(41.8), Inches.of(146.86), Inches.of(0));
  public static final Translation3d BlueClimbRight =
      new Translation3d(Inches.of(43.367), Inches.of(164.4), Inches.of(0));
  public static final Translation3d BlueClimbFarRight =
      new Translation3d(Inches.of(41.8), Inches.of(170.96), Inches.of(0));

  private static final AprilTagFields AprilTagField = AprilTagFields.k2026RebuiltAndymark;
  public static AprilTagFieldLayout AprilTagLayout;
  public static FieldPose2026 Outpost;
  public static FieldPose2026 HubCenter;
  public static FieldPose2026 AllianceBallBox;
  public static FieldPose2026 ClimbFarLeftBar;
  public static FieldPose2026 ClimbLeftBar;
  public static FieldPose2026 ClimbMiddle;
  public static FieldPose2026 ClimbRightBar;
  public static FieldPose2026 ClimbFarRightBar;

  public FieldPose2026(Alliance defaultAlliance, String name, Pose3d defaultPose) {
    super(Midpoint, defaultAlliance, name, defaultPose);
  }

  public FieldPose2026(Alliance defaultAlliance, String name, Pose2d defaultPose) {
    super(Midpoint, defaultAlliance, name, new Pose3d(defaultPose));
  }

  /**
   * Required function before using any field points. Necessary because Java's static function
   * initializers will fail to load any of it. Values will be null. Run this at runtime to load the
   * data files early in the process.
   *
   * <p>Unfortunately, this means we can't make these fields final. You can thank Java for that.
   *
   * @throws IOException
   */
  public static void initializeFieldPoses() throws IOException {
    AprilTagLayout = new AprilTagFieldLayout(AprilTagField.m_resourceFile);
    Midpoint =
        new Translation2d(
            AprilTagLayout.getFieldLength() / 2.0, AprilTagLayout.getFieldWidth() / 2.0);
    // Individual Field Poses
    HubCenter =
        new FieldPose2026(Alliance.Blue, "HubCenter", new Pose3d(BlueHubCenter, new Rotation3d()));
    Outpost = new FieldPose2026(Alliance.Blue, "Outpost", AprilTagLayout.getTagPose(29).get());
    AllianceBallBox =
        new FieldPose2026(
            Alliance.Blue, "AllianceBallBox", new Pose3d(BlueAllianceBallBox, new Rotation3d()));
    // Climber positions
    ClimbFarLeftBar =
        new FieldPose2026(
            Alliance.Blue, "FarLeftClimb", new Pose3d(BlueClimbFarLeft, new Rotation3d()));
    ClimbLeftBar =
        new FieldPose2026(Alliance.Blue, "LeftClimb", new Pose3d(BlueClimbLeft, new Rotation3d()));
    ClimbMiddle =
        new FieldPose2026(
            Alliance.Blue, "MiddleClimb", new Pose3d(BlueClimbMiddle, new Rotation3d()));
    ClimbRightBar =
        new FieldPose2026(
            Alliance.Blue, "RightClimb", new Pose3d(BlueClimbRight, new Rotation3d()));
    ClimbFarLeftBar =
        new FieldPose2026(
            Alliance.Blue, "FarRightClimb", new Pose3d(BlueClimbFarRight, new Rotation3d()));
  }
}
