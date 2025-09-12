// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class StateHandlerConstants {
  public static final Pose2d targetPose = new Pose2d(0.81, 7.56, Rotation2d.fromDegrees(-54.69));

  public static final CommandXboxController controller = new CommandXboxController(0);

  public static void rumbleControllers() {
    controller.setRumble(RumbleType.kBothRumble, 1.0);
  }

  public static void stopRumble() {
    controller.setRumble(RumbleType.kBothRumble, 0.0);
  }

  public static Command rumbleController(double strength, double time, Subsystem subsystem) {
    return Commands.run(() -> controller.setRumble(RumbleType.kBothRumble, strength), subsystem)
        .withTimeout(time);
  }
}
