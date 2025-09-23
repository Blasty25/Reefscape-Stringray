// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autoAlign;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutoAlignConstants {

  public static int getClosestTagId(Pose2d robotPose) {
    int closestId = -1;
    double closestDist = Double.MAX_VALUE;

    AprilTagFieldLayout layout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    for (int id = 6; id <= 11; id++) {
      Optional<Pose3d> tagPose3d = layout.getTagPose(id);
      if (tagPose3d.isEmpty()) continue;

      Pose2d tagPose2d = tagPose3d.get().toPose2d();
      double dist = robotPose.getTranslation().getDistance(tagPose2d.getTranslation());

      if (dist < closestDist) {
        closestDist = dist;
        closestId = id;
      }
    }

    return closestId;
  }

  public static List<Pose2d> leftPersPose = new ArrayList<>();
  public static List<Pose2d> rightPersPose = new ArrayList<>();
  public static List<Pose2d> algaePose = new ArrayList<>();
  public static List<Pose2d> cagePoses = new ArrayList<>();

  public static LoggedTunableNumber xP = new LoggedTunableNumber("AutoAlign/XDrive/kP", 5.0);
  public static LoggedTunableNumber xI = new LoggedTunableNumber("AutoAlign/XDrive/kI", 0.0);
  public static LoggedTunableNumber xD = new LoggedTunableNumber("AutoAlign/XDrive/kD", 0.0);

  public static LoggedTunableNumber yP = new LoggedTunableNumber("AutoAlign/yDrive/kP", 4.0);
  public static LoggedTunableNumber yI = new LoggedTunableNumber("AutoAlign/yDrive/kI", 0.0);
  public static LoggedTunableNumber yD = new LoggedTunableNumber("AutoAlign/yDrive/kD", 0.0);

  public static LoggedTunableNumber zP = new LoggedTunableNumber("AutoAlign/zDrive/kP", 8.0);
  public static LoggedTunableNumber zI = new LoggedTunableNumber("AutoAlign/zDrive/kI", 0.0);
  public static LoggedTunableNumber zD = new LoggedTunableNumber("AutoAlign/zDrive/kD", 0.0);

  public static void setupAutoAlignment() {
    leftPersPose.add(new Pose2d(3.17, 4.31, Rotation2d.fromDegrees(0)));
    leftPersPose.add(new Pose2d(3.85, 5.31, Rotation2d.fromDegrees(-62.05)));
    leftPersPose.add(new Pose2d(5.02, 5.25, Rotation2d.fromDegrees(-118.6)));
    leftPersPose.add(new Pose2d(5.84, 4.21, Rotation2d.fromDegrees(180.0)));
    leftPersPose.add(new Pose2d(5.37, 2.94, Rotation2d.fromDegrees(126.79)));
    leftPersPose.add(new Pose2d(3.59, 2.95, Rotation2d.fromDegrees(55.92)));

    rightPersPose.add(new Pose2d(3.17, 3.84, Rotation2d.fromDegrees(0)));
    rightPersPose.add(new Pose2d(3.57, 5.16, Rotation2d.fromDegrees(-62.05)));
    rightPersPose.add(new Pose2d(5.36, 5.11, Rotation2d.fromDegrees(-118.6)));
    rightPersPose.add(new Pose2d(5.85, 3.92, Rotation2d.fromDegrees(180.0)));
    rightPersPose.add(new Pose2d(5.12, 2.75, Rotation2d.fromDegrees(124.79)));
    rightPersPose.add(new Pose2d(3.89, 2.76, Rotation2d.fromDegrees(62.11)));

    algaePose.add(new Pose2d(3.08, 4.01, Rotation2d.fromDegrees(0.0)));
    algaePose.add(new Pose2d(3.78, 5.22, Rotation2d.fromDegrees(-59.49)));
    algaePose.add(new Pose2d(5.23, 5.33, Rotation2d.fromDegrees(-120.29)));
    algaePose.add(new Pose2d(5.94, 4.06, Rotation2d.fromDegrees(180.0)));
    algaePose.add(new Pose2d(5.13, 2.81, Rotation2d.fromDegrees(119.17)));
    algaePose.add(new Pose2d(3.75, 2.77, Rotation2d.fromDegrees(58.16)));

    cagePoses.add(new Pose2d(8.02, 7.21, new Rotation2d()));
    cagePoses.add(new Pose2d(8.02, 6.19, new Rotation2d()));
    cagePoses.add(new Pose2d(8.02, 5.09, new Rotation2d()));
  }
}
