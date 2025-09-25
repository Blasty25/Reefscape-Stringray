package frc.robot.subsystems.autoAlign;

import static frc.robot.subsystems.autoAlign.AutoAlignConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoint;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoAlign extends SubsystemBase {

  private PIDController xPID =
      new PIDController(xP.getAsDouble(), xI.getAsDouble(), xD.getAsDouble());
  private PIDController yPID =
      new PIDController(yP.getAsDouble(), yI.getAsDouble(), yD.getAsDouble());
  private ProfiledPIDController zPID =
      new ProfiledPIDController(
          zP.getAsDouble(),
          zI.getAsDouble(),
          zD.getAsDouble(),
          new TrapezoidProfile.Constraints(
              DriveCommands.ANGLE_MAX_VELOCITY, DriveCommands.ANGLE_MAX_ACCELERATION));

  public AutoAlign() {
    setupAutoAlignment();
    zPID.enableContinuousInput(-Math.PI, Math.PI);
    xPID.setTolerance(0.01);
    yPID.setTolerance(0.01);
    zPID.setTolerance(0.01);
  }

  public Command driveToAlignWithReef(Drive drive, boolean leftOrNot, ElevatorSetpoint setpoints) {
    Pose2d target =
        leftOrNot ? drive.getPose().nearest(leftPersPose) : drive.getPose().nearest(rightPersPose);
    if (setpoints == ElevatorSetpoint.L4) {
      Transform2d relativeTransform =
          new Transform2d(new Translation2d(0.0, -0.5), new Rotation2d());
      target = drive.getPose().plus(relativeTransform);
      Logger.recordOutput("AutoAlign/L4Pose", true);
    }
    Logger.recordOutput("AutoAlign/L4Pose", false);

    Pose2d finalTarget = target;
    return Commands.run(
            () -> {
              Logger.recordOutput("AutoAlign/TargetPose", finalTarget);
              Logger.recordOutput("AutoAlign/xPID", xPID.getError());
              Logger.recordOutput("AutoAlign/yPID", yPID.getError());

              ChassisSpeeds driveToPoseSpeeds =
                  new ChassisSpeeds(
                      xPID.calculate(drive.getPose().getX(), finalTarget.getX()),
                      yPID.calculate(drive.getPose().getY(), finalTarget.getY()),
                      zPID.calculate(
                          drive.getRotation().getRadians(),
                          finalTarget.getRotation().getRadians()));

              // Set Chassis Speeds
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveToPoseSpeeds,
                      isFlipped
                          ? drive
                              .getRotation()
                              .plus(AllianceFlipUtil.apply(new Rotation2d(Math.PI)))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              xPID.reset();
              yPID.reset();
              zPID.reset(drive.getRotation().getRadians());
            })
        .until(() -> AutoAlign.isNear(finalTarget, drive.getPose()))
        .finallyDo(() -> drive.stop());
  }

  public Command driveToAlgaePose(Drive drive) {
    Pose2d target = drive.getPose().nearest(algaePose);
    return Commands.run(
            () -> {
              Logger.recordOutput("AutoAlign/TargetPose", target);
              Logger.recordOutput("AutoAlign/xPID", xPID.getError());
              Logger.recordOutput("AutoAlign/yPID", yPID.getError());

              ChassisSpeeds driveToPoseSpeeds =
                  new ChassisSpeeds(
                      xPID.calculate(drive.getPose().getX(), target.getX()),
                      yPID.calculate(drive.getPose().getY(), target.getY()),
                      zPID.calculate(
                          drive.getRotation().getRadians(), target.getRotation().getRadians()));

              // Set Chassis Speeds
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveToPoseSpeeds,
                      isFlipped
                          ? drive
                              .getRotation()
                              .plus(AllianceFlipUtil.apply(new Rotation2d(Math.PI)))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              xPID.reset();
              yPID.reset();
              zPID.reset(drive.getRotation().getRadians());
            })
        .until(() -> AutoAlign.isNear(target, drive.getPose()))
        .finallyDo(() -> drive.stop());
  }

  public Command driveToPreSelectedPose(Drive drive, Pose2d pose) {
    return Commands.run(
            () -> {
              Logger.recordOutput("AutoAlign/TargetPose", pose);
              Logger.recordOutput("AutoAlign/xPID", xPID.getError());
              Logger.recordOutput("AutoAlign/yPID", yPID.getError());

              ChassisSpeeds driveToPoseSpeeds =
                  new ChassisSpeeds(
                      xPID.calculate(drive.getPose().getX(), pose.getX()),
                      yPID.calculate(drive.getPose().getY(), pose.getY()),
                      zPID.calculate(
                          drive.getRotation().getRadians(), pose.getRotation().getRadians()));

              // Set Chassis Speeds
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveToPoseSpeeds,
                      isFlipped
                          ? drive
                              .getRotation()
                              .plus(AllianceFlipUtil.apply(new Rotation2d(Math.PI)))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              xPID.reset();
              yPID.reset();
              zPID.reset(drive.getRotation().getRadians());
            })
        .until(() -> AutoAlign.isNear(pose, drive.getPose()))
        .finallyDo(() -> drive.stop());
  }

  public Command driveToCage(Drive drive) {
    Pose2d pose = drive.getPose().nearest(cagePoses);
    return Commands.run(
            () -> {
              Logger.recordOutput("AutoAlign/ClimbPose", pose);
              Logger.recordOutput("AutoAlign/xPID", xPID.getError());
              Logger.recordOutput("AutoAlign/yPID", yPID.getError());

              ChassisSpeeds driveToPoseSpeeds =
                  new ChassisSpeeds(
                      xPID.calculate(drive.getPose().getX(), pose.getX()),
                      yPID.calculate(drive.getPose().getY(), pose.getY()),
                      zPID.calculate(
                          drive.getRotation().getRadians(), pose.getRotation().getRadians()));

              // Set Chassis Speeds
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveToPoseSpeeds,
                      isFlipped
                          ? drive
                              .getRotation()
                              .plus(AllianceFlipUtil.apply(new Rotation2d(Math.PI)))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              xPID.reset();
              yPID.reset();
              zPID.reset(drive.getRotation().getRadians());
            })
        .until(() -> AutoAlign.isNear(pose, drive.getPose()))
        .finallyDo(() -> drive.stop());
  }

  // Experimental Auto Aligning using Vision
  public Command visionAutoAlign(Drive drive) {
    return Commands.run(
            () -> {
              int tagId = AutoAlignConstants.getClosestTagId(drive.getPose());

              Pose3d allinment = aprilTagLayout.getTagPose(tagId).orElse(null);

              Pose2d pose = allinment.toPose2d();

              Logger.recordOutput("AutoAlign/ClimbPose", pose);
              Logger.recordOutput("AutoAlign/xPID", xPID.getError());
              Logger.recordOutput("AutoAlign/yPID", yPID.getError());

              ChassisSpeeds driveToPoseSpeeds =
                  new ChassisSpeeds(
                      xPID.calculate(drive.getPose().getX(), pose.getX()),
                      yPID.calculate(drive.getPose().getY(), pose.getY()),
                      zPID.calculate(
                          drive.getRotation().getRadians(), pose.getRotation().getRadians()));

              // Set Chassis Speeds
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveToPoseSpeeds,
                      isFlipped
                          ? drive
                              .getRotation()
                              .plus(AllianceFlipUtil.apply(new Rotation2d(Math.PI)))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              xPID.reset();
              yPID.reset();
              zPID.reset(drive.getRotation().getRadians());
            })
        .until(() -> xPID.atSetpoint() && yPID.atSetpoint() && zPID.atSetpoint())
        .finallyDo(() -> drive.stop());
  }

  public static boolean isNear(Pose2d target, Pose2d actual) {
    return MathUtil.isNear(0, actual.relativeTo(target).getTranslation().getNorm(), 0.025)
        && MathUtil.isNear(
            target.getRotation().getRadians(), actual.getRotation().getRadians(), 0.025);
  }

  @Override
  public void periodic() {}
}
