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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoint;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoAlign extends SubsystemBase {

  public static Pose2d FINAL_CORAL_POSE;
  public static Pose2d FINAL_ALGAE_POSE;
  public static Pose2d FINAL_CAGE_POSE;
  public static double L4_BACK_DISTANCE = 2.5; //INCHS

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
    xPID.setTolerance(0.03);
    yPID.setTolerance(0.03);
    zPID.setTolerance(0.03);
  }

  public Command driveToAlignWithReef(Drive drive, boolean leftOrNot, Elevator elevator) {
    return Commands.startRun(
            () -> {
              Pose2d target =
                  leftOrNot
                      ? drive.getPose().nearest(leftPersPose)
                      : drive.getPose().nearest(rightPersPose);

              if (elevator.getNextExpectedSetpoint() == ElevatorSetpoint.L4) {
                Logger.recordOutput("AutoAlign/HasL4", true);

                double backMeters = -Units.inchesToMeters(L4_BACK_DISTANCE); // negative = backwards
                Transform2d backwardsTransform =
                    new Transform2d(new Translation2d(backMeters, 0.0), new Rotation2d());

                target = target.plus(backwardsTransform);
              } else {
                Logger.recordOutput("AutoAlign/HasL4", false);
              }

              FINAL_CORAL_POSE = target;
            },
            () -> {
              Logger.recordOutput("AutoAlign/TargetPose", FINAL_CORAL_POSE);
              Logger.recordOutput("AutoAlign/xPID", xPID.getError());
              Logger.recordOutput("AutoAlign/yPID", yPID.getError());

              ChassisSpeeds driveToPoseSpeeds =
                  new ChassisSpeeds(
                      xPID.calculate(drive.getPose().getX(), FINAL_CORAL_POSE.getX()),
                      yPID.calculate(drive.getPose().getY(), FINAL_CORAL_POSE.getY()),
                      zPID.calculate(
                          drive.getRotation().getRadians(),
                          FINAL_CORAL_POSE.getRotation().getRadians()));

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
        .until(() -> AutoAlign.isNear(FINAL_CORAL_POSE, drive.getPose()))
        .finallyDo(() -> drive.stopWithX());
  }

  public Command driveToAlgaePose(Drive drive) {
    return Commands.startRun(
            () -> {
              FINAL_ALGAE_POSE = drive.getPose().nearest(algaePose);
            },
            () -> {
              Logger.recordOutput("AutoAlign/TargetPose", FINAL_ALGAE_POSE);
              Logger.recordOutput("AutoAlign/xPID", xPID.getError());
              Logger.recordOutput("AutoAlign/yPID", yPID.getError());

              ChassisSpeeds driveToPoseSpeeds =
                  new ChassisSpeeds(
                      xPID.calculate(drive.getPose().getX(), FINAL_ALGAE_POSE.getX()),
                      yPID.calculate(drive.getPose().getY(), FINAL_ALGAE_POSE.getY()),
                      zPID.calculate(
                          drive.getRotation().getRadians(),
                          FINAL_ALGAE_POSE.getRotation().getRadians()));

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
        .until(() -> AutoAlign.isNear(FINAL_ALGAE_POSE, drive.getPose()))
        .finallyDo(() -> drive.stopWithX());
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
    return Commands.startRun(
            () -> {
              FINAL_CAGE_POSE = drive.getPose().nearest(cagePoses);
            },
            () -> {
              Logger.recordOutput("AutoAlign/ClimbPose", FINAL_CAGE_POSE);
              Logger.recordOutput("AutoAlign/xPID", xPID.getError());
              Logger.recordOutput("AutoAlign/yPID", yPID.getError());

              ChassisSpeeds driveToPoseSpeeds =
                  new ChassisSpeeds(
                      xPID.calculate(drive.getPose().getX(), FINAL_CAGE_POSE.getX()),
                      yPID.calculate(drive.getPose().getY(), FINAL_CAGE_POSE.getY()),
                      zPID.calculate(
                          drive.getRotation().getRadians(),
                          FINAL_CAGE_POSE.getRotation().getRadians()));

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
        .until(() -> AutoAlign.isNear(FINAL_CAGE_POSE, drive.getPose()))
        .finallyDo(() -> drive.stopWithX());
  }

  // Experimental Auto Aligning using Vision
  // DEBUGGING NOT OFFICIAL USE
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
