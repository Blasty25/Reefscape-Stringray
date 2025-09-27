package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static List<Pose2d> blueReefTagPoses =
      List.of(
          aprilTagLayout.getTagPose(17).get().toPose2d(),
          aprilTagLayout.getTagPose(18).get().toPose2d(),
          aprilTagLayout.getTagPose(19).get().toPose2d(),
          aprilTagLayout.getTagPose(20).get().toPose2d(),
          aprilTagLayout.getTagPose(21).get().toPose2d(),
          aprilTagLayout.getTagPose(22).get().toPose2d());

  public static List<Pose2d> redReefTagPoses =
      List.of(
          aprilTagLayout.getTagPose(6).get().toPose2d(),
          aprilTagLayout.getTagPose(7).get().toPose2d(),
          aprilTagLayout.getTagPose(8).get().toPose2d(),
          aprilTagLayout.getTagPose(9).get().toPose2d(),
          aprilTagLayout.getTagPose(10).get().toPose2d(),
          aprilTagLayout.getTagPose(11).get().toPose2d());

  // Camera names, must match names configured on coprocessor
  public static String camLeft = "CamLeft";
  public static String camRight = "CamRight";

  // Robot to camera transforms, measured empirically
  public static Transform3d robotToLeftCam =
      new Transform3d(
          Units.inchesToMeters(12.066),
          Units.inchesToMeters(11.906),
          Units.inchesToMeters(8.355),
          new Rotation3d(0.0, -Units.degreesToRadians(13.125000), Units.degreesToRadians(-30)));
  public static Transform3d robotToRightCam =
      new Transform3d(
          Units.inchesToMeters(12.066),
          Units.inchesToMeters(-11.906),
          Units.inchesToMeters(8.355),
          new Rotation3d(0.0, -Units.degreesToRadians(13.125000), Units.degreesToRadians(30)));
  // Basic filtering thresholds
  public static double maxSingleTagAmbiguity = 0.4;
  public static double maxZErrorMeters = 0.1;
  public static double maxSingleTagDistanceMeters = 1.25;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double trigLinearStdDevBaseline = 0.1; // Meters
  public static double trigAngularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  public static double multitagLinearStdDevBaseline = 0.35; // Meters
  public static double multitagAngularStdDevBaseline = 0.14; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera FL
        1.0, // Camera FR
      };
}
