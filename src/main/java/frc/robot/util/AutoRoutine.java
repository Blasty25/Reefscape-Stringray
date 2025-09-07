package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.autoAlign.AutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDConstants;
import frc.robot.subsystems.outtake.Outtake;

public class AutoRoutine extends SubsystemBase {

    private Drive drive;
    private AutoAlign autoAlign;
    private Hopper hopper;
    private Outtake outtake;
    private Elevator elevator;
    private LED led;

    public AutoRoutine(
            Drive drive,
            AutoAlign autoAlign,
            Hopper hopper,
            Outtake outtake,
            Elevator elevator,
            LED led) {
        this.drive = drive;
        this.autoAlign = autoAlign;
        this.hopper = hopper;
        this.outtake = outtake;
        this.elevator = elevator;
        this.led = led;
    }

    public Command autoCoralL4LeftSide() {
        return Commands.sequence(
                autoAlign.driveToPreSelectedPose(
                        drive, new Pose2d(5.06, 5.35, Rotation2d.fromDegrees(-124.05))),
                elevator.setTarget(ElevatorSetpoints.L4),
                // led.set(LEDConstants.shootCoralAnimation),
                Commands.waitSeconds(0.8),
                outtake.shoot(),
                Commands.waitSeconds(0.5),
                elevator.setTarget(ElevatorSetpoints.INTAKE),
                // led.set(LEDConstants.whiteStrobe),
                autoAlign.driveToPreSelectedPose(
                        drive, new Pose2d(1.16, 6.96, Rotation2d.fromDegrees(-57.77))),
                hopper.autoIntake(outtake),
                autoAlign.driveToPreSelectedPose(
                        drive, new Pose2d(3.88, 5.28, Rotation2d.fromDegrees(-60.47))),
                elevator.setTarget(ElevatorSetpoints.L4),
                // led.set(LEDConstants.shootCoralAnimation),
                Commands.waitSeconds(0.8),
                outtake.shoot(),
                elevator.setTarget(ElevatorSetpoints.INTAKE),
                // led.set(LEDConstants.whiteStrobe),
                autoAlign.driveToPreSelectedPose(
                        drive, new Pose2d(1.16, 6.96, Rotation2d.fromDegrees(-57.77))),
                hopper.autoIntake(outtake),
                autoAlign.driveToPreSelectedPose(
                        drive, new Pose2d(3.61, 5.07, Rotation2d.fromDegrees(-59.06))),
                elevator.setTarget(ElevatorSetpoints.L4),
                // led.set(LEDConstants.shootCoralAnimation),
                Commands.waitSeconds(0.8),
                outtake.shoot(),
                elevator.setTarget(ElevatorSetpoints.INTAKE),
                // led.set(LEDConstants.whiteStrobe),
                autoAlign.driveToPreSelectedPose(
                        drive, new Pose2d(1.16, 6.96, Rotation2d.fromDegrees(-57.77))),
                hopper.autoIntake(outtake),
                autoAlign.driveToPreSelectedPose(drive, new Pose2d(3.13, 4.19, new Rotation2d())),
                elevator.setTarget(ElevatorSetpoints.L4),
                // led.set(LEDConstants.shootCoralAnimation),
                Commands.waitSeconds(0.8),
                outtake.shoot(),
                elevator.setTarget(ElevatorSetpoints.INTAKE));
    }

    /*
     * 1st Pose 5.06, 5.35, Rotation(-124.05)
     * 2nd Pose 1.16, 6.96 Rotation(-57.77)
     * 3rd Pose 3.88, 5.28, Rotation(-60.47)
     * 4th Pose 1.16, 6.96, Rotation(-57.77)
     * 5th Pose 3.61, 5.07, Rotation(-59.06)
     * 6th Pose 1.16, 6.96, Rotation(-57.77)
     * 7th Pose 3.13, 4.19, Rotation(0.14)
     */
}
