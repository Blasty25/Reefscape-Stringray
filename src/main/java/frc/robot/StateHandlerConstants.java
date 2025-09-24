// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class StateHandlerConstants {
  public static final Pose2d leftIntakePose =
      new Pose2d(0.81, 7.56, Rotation2d.fromDegrees(-54.69));
  public static final Pose2d rightIntakePose =
      new Pose2d(0.81, 0.38, Rotation2d.fromDegrees(54.69));

  public static final Pose2d reefCenterPose = new Pose2d(4.45, 4.05, new Rotation2d());

  public static final CommandXboxController controller = new CommandXboxController(0);
  public static final CommandXboxController operatorOveride = new CommandXboxController(1);

  public static Command rumble(double value, double time) {
    return Commands.runEnd(
            () -> controller.setRumble(RumbleType.kBothRumble, 1),
            () -> controller.setRumble(RumbleType.kBothRumble, 0))
        .raceWith(Commands.waitSeconds(time));
  }
}
