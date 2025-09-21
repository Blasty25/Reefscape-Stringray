package frc.robot;

import static frc.robot.StateHandlerConstants.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotState;
import frc.robot.subsystems.autoAlign.AutoAlign;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoint;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.outtake.Outtake;
import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class StateMachine extends SubsystemBase {

    // Subsystem references
    private Drive drive;
    private Elevator elevator;
    private Outtake outtake;
    private CommandXboxController driver;
    private AutoAlign autoAlign;
    private Gripper gripper;
    private Climb climb;
    private LED led;
    private Hopper hopper;
    private CommandXboxController operatorOveride;

    // Maps to store triggers for state requests and state-based conditions
    private Map<RobotState, Trigger> stateRequests = new EnumMap<>(RobotState.class);
    private Map<RobotState, Trigger> stateTriggers = new EnumMap<>(RobotState.class);

    // Logging the current and previous robot state
    @AutoLogOutput(key = "RobotState/CurrentState")
    private RobotState state = RobotState.Idle;

    @AutoLogOutput(key = "RobotState/PreviousState")
    private RobotState previousState = RobotState.Idle;

    // Constructor: assign subsystems and configure triggers
    public StateMachine(
            Drive drive,
            Elevator elevator,
            Outtake outtake,
            Gripper gripper,
            Hopper hopper,
            Climb climb,
            AutoAlign autoAlign,
            LED led,
            CommandXboxController controller,
            CommandXboxController operatorOveride) {

        this.drive = drive;
        this.elevator = elevator;
        this.outtake = outtake;
        this.driver = controller;
        this.autoAlign = autoAlign;
        this.gripper = gripper;
        this.climb = climb;
        this.led = led;
        this.hopper = hopper;
        this.operatorOveride = operatorOveride;

        // Assign controller buttons to state requests
        stateRequests.put(RobotState.Idle, driver.a());
        stateRequests.put(RobotState.Intake, driver.leftTrigger());
        stateRequests.put(RobotState.SetElevatorSetpoint, driver.povRight());
        stateRequests.put(RobotState.Manual_Score, driver.povLeft());
        stateRequests.put(RobotState.Shoot, driver.rightTrigger());
        stateRequests.put(RobotState.Eject, driver.rightTrigger());
        stateRequests.put(RobotState.Climb_Ready, driver.povDown());
        stateRequests.put(RobotState.Climb_Stow, driver.povUp());
        stateRequests.put(RobotState.Climb_Pull, driver.rightTrigger());
        stateRequests.put(RobotState.Manual_Elevator, operatorOveride.a());

        // Initialize triggers for each state: true if robot is in that state and
        // enabled
        for (RobotState state : RobotState.values()) {
            stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
        }

        // Set up all state-based triggers and commands
        enableStateSetup();
    }

    // Configure commands associated with each robot state
    public void enableStateSetup() {

        // Operator override to force Idle state
        operatorOveride.x().onTrue(forceState(RobotState.Idle));

        // Stop gripper voltage when Idle and nothing detected
        stateTriggers
                .get(RobotState.Idle)
                .and(() -> !gripper.getDualDetected())
                .onTrue(gripper.setVoltage(0.0));

        // Return to Idle if Shoot state but no outtake detected
        stateTriggers
                .get(RobotState.Shoot)
                .and(() -> !outtake.isDetected())
                .onTrue(forceState(RobotState.Idle));

        // Manual Score state request from POV right
        stateTriggers
                .get(RobotState.Idle)
                .and(driver.povRight())
                .onTrue(forceState(RobotState.Manual_Score));

        // Pre-Algae state triggered by right trigger
        stateTriggers
                .get(RobotState.Idle)
                .and(driver.rightTrigger())
                .onTrue(forceState(RobotState.Pre_Algae));

        // Move to SetElevatorSetpoint if outtake detected, otherwise stay Idle
        stateTriggers
                .get(RobotState.Idle)
                .and(() -> outtake.isDetected())
                .onTrue(forceState(RobotState.SetElevatorSetpoint))
                .onFalse(forceState(RobotState.Idle));

        // Algae armed state if gripper detects dual
        stateTriggers
                .get(RobotState.Algae_Intake)
                .and(() -> gripper.getDualDetected())
                .onTrue(forceState(RobotState.Algae_Armed));

        stateTriggers
                .get(RobotState.Algae_Armed)
                .and(() -> !gripper.getDualDetected())
                .onTrue(forceState(RobotState.Idle));

        // Climb-related triggers
        stateTriggers
                .get(RobotState.Idle)
                .and(stateRequests.get(RobotState.Climb_Ready))
                .onTrue(forceState(RobotState.Climb_Ready));

        stateTriggers
                .get(RobotState.Climb_Ready)
                .and(stateRequests.get(RobotState.Climb_Stow))
                .onTrue(forceState(RobotState.Climb_Stow));

        stateTriggers
                .get(RobotState.Climb_Ready)
                .and(stateRequests.get(RobotState.Climb_Pull))
                .onTrue(forceState(RobotState.Climb_Pull));

        // Elevator setpoint sequences
        stateTriggers
                .get(RobotState.Idle)
                .onTrue(
                        Commands.sequence(
                                elevator.setTarget(ElevatorSetpoint.INTAKE), elevator.setExtension()));

        stateTriggers
                .get(RobotState.SetElevatorSetpoint)
                .onTrue(
                        Commands.sequence(elevator.setTarget(ElevatorSetpoint.L1), elevator.setExtension()));

        // Shoot trigger when elevator setpoint active
        stateTriggers
                .get(RobotState.SetElevatorSetpoint)
                .and(driver.rightTrigger())
                .onTrue(outtake.shoot());

        // Intake command while Idle
        stateTriggers
                .get(RobotState.Idle)
                .and(stateRequests.get(RobotState.Intake))
                .onTrue(Commands.parallel(hopper.intake(), forceState(RobotState.Intake)));

        // Transition to SetElevatorSetpoint if outtake is detected during Intake
        stateTriggers
                .get(RobotState.Intake)
                .and(() -> outtake.isDetected())
                .onTrue(forceState(RobotState.SetElevatorSetpoint));

        // Reset gyro command while Idle
        stateTriggers
                .get(RobotState.Idle)
                .and(driver.b())
                .onTrue(
                        Commands.runOnce(
                                () -> drive.setPose(
                                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                drive)
                                .ignoringDisable(true));

        // Hopper control during Intake using triggers
        stateTriggers
                .get(RobotState.Intake)
                .and(driver.rightTrigger())
                .whileTrue(
                        Commands.startEnd(
                                () -> hopper.setTrackPercent(-1.0), () -> hopper.setTrackPercent(0), hopper));

        stateTriggers
                .get(RobotState.Intake)
                .and(driver.leftTrigger())
                .whileTrue(
                        Commands.startEnd(
                                () -> hopper.setTrackPercent(1.0), () -> hopper.setTrackPercent(0), hopper));

        // Elevator setpoint buttons (L2-L4) and force Shoot state
        stateTriggers
                .get(RobotState.SetElevatorSetpoint)
                .and(driver.b())
                .onTrue(
                        Commands.parallel(
                                elevator.setTarget(ElevatorSetpoint.L2), forceState(RobotState.Shoot)));

        stateTriggers
                .get(RobotState.SetElevatorSetpoint)
                .and(driver.x())
                .onTrue(
                        Commands.parallel(
                                elevator.setTarget(ElevatorSetpoint.L3), forceState(RobotState.Shoot)));

        stateTriggers
                .get(RobotState.SetElevatorSetpoint)
                .and(driver.y())
                .onTrue(
                        Commands.parallel(
                                elevator.setTarget(ElevatorSetpoint.L4), forceState(RobotState.Shoot)));

        // Shoot state right trigger sequence
        stateTriggers
                .get(RobotState.Shoot)
                .and(driver.rightTrigger())
                .onTrue(
                        Commands.sequence(
                                elevator.setExtension(),
                                Commands.waitUntil(() -> elevator.atSetpoint()),
                                outtake.shoot(),
                                Commands.waitSeconds(0.01)));

        // Auto-alignment during Shoot using bumpers
        stateTriggers
                .get(RobotState.Shoot)
                .and(driver.leftBumper())
                .onTrue(autoAlign.driveToAlignWithReef(drive, true, elevator.getSetpoint()));

        stateTriggers
                .get(RobotState.Shoot)
                .and(driver.rightBumper())
                .onTrue(autoAlign.driveToAlignWithReef(drive, false, elevator.getSetpoint()));

        // Pre-Algae alignment
        stateTriggers
                .get(RobotState.Pre_Algae)
                .onTrue(
                        Commands.parallel(
                                autoAlign.driveToAlgaePose(drive), forceState(RobotState.Algae_Setpoint)));

        // Algae setpoints (A2-A3) from driver buttons
        stateTriggers
                .get(RobotState.Algae_Setpoint)
                .and(driver.x())
                .onTrue(
                        Commands.parallel(
                                elevator.setTarget(ElevatorSetpoint.A3), forceState(RobotState.Algae_Intake)));

        stateTriggers
                .get(RobotState.Algae_Setpoint)
                .and(driver.b())
                .onTrue(
                        Commands.parallel(
                                elevator.setTarget(ElevatorSetpoint.A2), forceState(RobotState.Algae_Intake)));

        // Intake algae into gripper
        stateTriggers
                .get(RobotState.Algae_Intake)
                .and(driver.leftTrigger())
                .onTrue(gripper.setVoltage(GripperConstants.A23));

        // Move elevator back to L1 after algae armed
        stateTriggers.get(RobotState.Algae_Armed).onTrue((elevator.setTarget(ElevatorSetpoint.L1)));

        // Shoot algae using right trigger & left trigger
        stateTriggers
                .get(RobotState.Algae_Armed)
                .and(driver.rightTrigger())
                .and(driver.leftTrigger())
                .onTrue(
                        Commands.parallel(
                                gripper.setVoltage(GripperConstants.AP),
                                Commands.runOnce(() -> gripper.setSimDetected(false)),
                                Commands.runOnce(() -> System.out.println("Algae Shot")),
                                elevator.setTarget(ElevatorSetpoint.INTAKE),
                                forceState(RobotState.Idle)));

        // Shoot algae to barge using RT button
        stateTriggers
                .get(RobotState.Algae_Armed)
                .and(driver.rightTrigger())
                .onTrue(
                        Commands.sequence(
                                elevator.setTarget(ElevatorSetpoint.L4),
                                elevator.setExtension(),
                                gripper.setVoltage(GripperConstants.AN)));

        // Manual scoring elevator setpoints
        stateTriggers
                .get(RobotState.Manual_Score)
                .and(driver.y())
                .onTrue(Commands.parallel(elevator.setTarget(ElevatorSetpoint.L4)));
        stateTriggers
                .get(RobotState.Manual_Score)
                .and(driver.x())
                .onTrue(Commands.parallel(elevator.setTarget(ElevatorSetpoint.L3)));
        stateTriggers
                .get(RobotState.Manual_Score)
                .and(driver.b())
                .onTrue(Commands.parallel(elevator.setTarget(ElevatorSetpoint.L2)));
        stateTriggers
                .get(RobotState.Manual_Score)
                .and(driver.a())
                .onTrue(Commands.parallel(elevator.setTarget(ElevatorSetpoint.L1)));

        // Shoot coral during manual score
        stateTriggers
                .get(RobotState.Manual_Score)
                .and(driver.rightTrigger())
                .onTrue(outtake.shoot());

        // Climb state commands
        stateTriggers
                .get(RobotState.Climb_Pull)
                .onTrue(Commands.parallel(climb.setPosition(ClimbConstants.climbed)));
        stateTriggers
                .get(RobotState.Climb_Ready)
                .onTrue(Commands.parallel(climb.setPosition(ClimbConstants.ready)));
        // Auto Drive to Closest Cage
        stateTriggers
                .get(RobotState.Climb_Ready)
                .and(driver.leftTrigger())
                .onTrue(autoAlign.driveToCage(drive));
        stateTriggers
                .get(RobotState.Climb_Stow)
                .onTrue(
                        Commands.parallel(climb.setPosition(ClimbConstants.stow), forceState(RobotState.Idle)));

        /*
         * RobotCancelRequests -- Combination of Left or Right Triggers
         */

        // Eject corral while SetElevatorSetpoint and Eject requested
        stateTriggers
                .get(RobotState.SetElevatorSetpoint)
                .and(stateRequests.get(RobotState.Eject)) // Right Trigger
                .and(driver.a())
                .onTrue(outtake.ejectCorral());

        // Robot is in Intake and in order to change to Idle - Driver hits A
        stateTriggers
                .get(RobotState.Intake)
                .and(driver.a()) // Button A
                .onTrue(forceState(RobotState.Idle));

        // Robot is Auto Aligning for Algae, Cancel Pre_Algae - Driver hits LT
        stateTriggers
                .get(RobotState.Pre_Algae)
                .and(driver.leftTrigger()) // Left Trigger
                .onTrue(forceState(RobotState.Idle));

        // Elevator is at Algae Intake Height, Cancel Request is RT - Driver
        stateTriggers
                .get(RobotState.Algae_Setpoint)
                .and(stateRequests.get(RobotState.Eject)) // Right Trigger
                .onTrue(forceState(RobotState.Idle));

        // Robot is Intaking Algae using LT, Cancel Request is RT
        stateTriggers.get(RobotState.Algae_Intake)
                .and(stateRequests.get(RobotState.Eject)) // Right Trigger
                .onTrue(forceState(RobotState.Idle));
        
        //Robot has a Algae, Cancel Request is LT, to Drop Algae, and go back to Idle
        stateTriggers
            .get(RobotState.Algae_Armed)
            .and(driver.leftTrigger()) // Left Trigger
            .onTrue(Commands.parallel(gripper.setVoltage(12.0), forceState(RobotState.Idle)));
    }

    // Helper command to force a robot state change
    private Command forceState(RobotState nextState) {
        return Commands.runOnce(
                () -> {
                    System.out.println("Changing state to " + nextState);
                    led.setState(nextState);
                    previousState = state;
                    state = nextState;
                });
    }

    @Override
    public void periodic() {
        // Force Idle state when disabled
        if (DriverStation.isDisabled()) {
            forceState(RobotState.Idle).schedule();
        }

        // Logging: check if robot is within 2.5 units of reef center
        Logger.recordOutput(
                "Intake",
                StateHandlerConstants.reefCenterPose
                        .getTranslation()
                        .getDistance(drive.getPose().getTranslation()) <= 2.5);

        // Experimental endgame rumble
        if (Timer.getMatchTime() <= 15.0 && DriverStation.isTeleopEnabled()) {
            controller.setRumble(RumbleType.kBothRumble, 0.5);
        }
        if (Timer.getMatchTime() >= 10.0 && DriverStation.isTeleopEnabled()) {
            controller.setRumble(RumbleType.kBothRumble, 0.0);
        }
    }
}
