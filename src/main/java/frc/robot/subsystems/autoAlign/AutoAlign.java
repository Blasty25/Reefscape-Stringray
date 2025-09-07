package frc.robot.subsystems.autoAlign;

import static frc.robot.subsystems.autoAlign.AutoAlignConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
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

  private Pose2d target;
  private ElevatorSetpoints setpoint;

  public AutoAlign() {
    zPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void leftAlign(Drive drive, Pose2d target, ElevatorSetpoints setpoint) {
    System.out.println("Testing");
    this.target = target;
    this.setpoint = setpoint;

    if (setpoint.equals(ElevatorSetpoints.L4)) {}
  }

  public Command driveToPose(Drive drive, boolean leftOrNot, ElevatorSetpoints setpoints) {
    return Commands.run(
            () -> {
              Pose2d target;

              if (leftOrNot) {
                target = drive.getPose().nearest(leftPersPose);
              } else {
                target = drive.getPose().nearest(rightPersPose);
              }

              if (setpoints.equals(ElevatorSetpoints.L4)) {
                System.out.println("L4L4L4 L4 L4 L4");
                Translation2d translation =
                    new Translation2d(-0.5, 0.0).rotateBy(target.getRotation());

                Logger.recordOutput("AutoAlign/TargetPose", target);
                Logger.recordOutput("AutoAlign/xPID", xPID.getError());
                Logger.recordOutput("AutoAlign/yPID", yPID.getError());

                Pose2d finalL4Pose =
                    new Pose2d(
                        target.getX() + translation.getX(),
                        target.getY() + translation.getY(),
                        target.getRotation());

                ChassisSpeeds driveToPoseSpeeds =
                    new ChassisSpeeds(
                        xPID.calculate(drive.getPose().getX(), finalL4Pose.getX()),
                        yPID.calculate(drive.getPose().getY(), finalL4Pose.getY()),
                        zPID.calculate(
                            drive.getRotation().getRadians(),
                            finalL4Pose.getRotation().getRadians()));

                // Set Chassis Speeds
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveToPoseSpeeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
              }

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
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
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

  public Command driveToAlgaePose(Drive drive) {
    return Commands.run(
            () -> {
              Pose2d target = drive.getPose().nearest(algaePose);

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
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              xPID.reset();
              yPID.reset();
              zPID.reset(drive.getRotation().getRadians());
            })
        .until(() -> xPID.atSetpoint() && yPID.atSetpoint() && zPID.atSetpoint());
  }

  @Override
  public void periodic() {}
}
