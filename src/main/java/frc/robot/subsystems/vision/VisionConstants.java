// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // Camera names, must match names configured on coprocessor
  public static String leftCam = "CamLeft";
  public static String rightCam = "CamRight";

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
