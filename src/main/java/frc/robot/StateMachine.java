package frc.robot;

import static frc.robot.StateHandlerConstants.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
    stateRequests.put(RobotState.Manual_Score, operatorOveride.povLeft());
    stateRequests.put(RobotState.Shoot, driver.rightTrigger());
    stateRequests.put(RobotState.ClimbReady, driver.povDown());
    stateRequests.put(RobotState.ClimbStow, driver.povUp());
    stateRequests.put(RobotState.ClimbPull, driver.rightTrigger());

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
        .and(operatorOveride.povRight())
        .onTrue(forceState(RobotState.Manual_Score));

    // Pre-Algae state triggered by right trigger
    stateTriggers
        .get(RobotState.Idle)
        .and(driver.rightTrigger())
        .onTrue(forceState(RobotState.PreAlgae));

    // Move to SetElevatorSetpoint if outtake detected, otherwise stay Idle
    stateTriggers
        .get(RobotState.Idle)
        .and(() -> outtake.isDetected())
        .onTrue(forceState(RobotState.SetElevatorSetpoint))
        .onFalse(forceState(RobotState.Idle));

    // Algae armed state if gripper detects dual
    stateTriggers
        .get(RobotState.AlgaeIntake)
        .and(() -> gripper.getDualDetected())
        .onTrue(forceState(RobotState.AlgaeArmed));

    stateTriggers
        .get(RobotState.AlgaeArmed)
        .and(() -> !gripper.getDualDetected())
        .onTrue(forceState(RobotState.Idle));

    // Climb-related triggers
    stateTriggers
        .get(RobotState.Idle)
        .and(stateRequests.get(RobotState.ClimbReady))
        .onTrue(forceState(RobotState.ClimbReady));

    stateTriggers
        .get(RobotState.ClimbReady)
        .and(stateRequests.get(RobotState.ClimbStow))
        .onTrue(forceState(RobotState.ClimbStow));

    stateTriggers
        .get(RobotState.ClimbReady)
        .and(stateRequests.get(RobotState.ClimbPull))
        .onTrue(forceState(RobotState.ClimbPull));

    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(driver.povUp())
        .whileTrue(
            Commands.parallel(
                hopper.overideVoltage(-6), outtake.ejectCorral(), forceState(RobotState.Idle)));

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
        .onTrue(
            Commands.sequence(
                outtake.shoot(), Commands.waitSeconds(0.1), forceState(RobotState.Idle)));

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
        .and(driver.povRight())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
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
        .onTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> hopper.setTrackPercent(1.0), () -> hopper.setTrackPercent(0), hopper),
                outtake.intake()));

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
        .get(RobotState.PreAlgae)
        .onTrue(
            Commands.parallel(
                autoAlign.driveToAlgaePose(drive), forceState(RobotState.AlgaeSetpoint)));

    // Algae setpoints (A2-A3) from driver buttons
    stateTriggers
        .get(RobotState.AlgaeSetpoint)
        .and(driver.x())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoint.A3), forceState(RobotState.AlgaeIntake)));

    stateTriggers
        .get(RobotState.AlgaeSetpoint)
        .and(driver.b())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoint.A2), forceState(RobotState.AlgaeIntake)));

    // Intake algae into gripper
    stateTriggers
        .get(RobotState.AlgaeIntake)
        .onTrue(
            Commands.sequence(elevator.setExtension(), gripper.setVoltage(GripperConstants.A23)));

    // Move elevator back to L1 after algae armed
    stateTriggers
        .get(RobotState.AlgaeArmed)
        .onTrue(
            (Commands.sequence(elevator.setTarget(ElevatorSetpoint.L1), elevator.setExtension())));

    // Shoot algae using right trigger & left trigger
    stateTriggers
        .get(RobotState.AlgaeArmed)
        .and(driver.leftTrigger())
        .onTrue(
            Commands.parallel(
                gripper.setVoltage(GripperConstants.AP),
                Commands.runOnce(() -> System.out.println("Algae Shot")),
                elevator.setTarget(ElevatorSetpoint.INTAKE),
                forceState(RobotState.Idle)));

    // Shoot algae to barge using RT button | Timing has to be tuned
    stateTriggers
        .get(RobotState.AlgaeArmed)
        .and(driver.rightTrigger())
        .onTrue(
            Commands.sequence(
                elevator.setTarget(ElevatorSetpoint.L4),
                elevator.setExtension(),
                Commands.waitSeconds(0.6),
                gripper.setVoltage(GripperConstants.AN)));

    // stateTriggers
    //     .get(RobotState.AlgaeArmed)
    //     .onTrue(
    //         Commands.run(
    //             () -> {
    //               boolean inRange =
    //                   gripper.isRobotInFrontOfBarge(drive.getPose()); // 1.5 == 1.5 Meters
    //               Logger.recordOutput("/LED/inRange", inRange);
    //               if (inRange) {
    //                 led.wave(Color.kBlue, Color.kYellow, 20, 3);
    //               }
    //             },
    //             gripper));

    // Manual scoring elevator setpoints -- More so for testing!
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(operatorOveride.y())
        .onTrue(
            Commands.parallel(elevator.setTarget(ElevatorSetpoint.L4), elevator.setExtension()));
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(operatorOveride.x())
        .onTrue(
            Commands.parallel(elevator.setTarget(ElevatorSetpoint.L3), elevator.setExtension()));
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(operatorOveride.b())
        .onTrue(
            Commands.parallel(elevator.setTarget(ElevatorSetpoint.L2), elevator.setExtension()));
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(operatorOveride.a())
        .onTrue(
            Commands.parallel(elevator.setTarget(ElevatorSetpoint.L1), elevator.setExtension()));
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(operatorOveride.povDown())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoint.INTAKE), elevator.setExtension()));

    // Shoot coral during manual score
    stateTriggers.get(RobotState.Manual_Score).and(driver.rightTrigger()).onTrue(outtake.shoot());

    // Climb state commands
    stateTriggers
        .get(RobotState.ClimbPull)
        .onTrue(Commands.parallel(climb.setPosition(ClimbConstants.climbed)));
    stateTriggers
        .get(RobotState.ClimbReady)
        .onTrue(Commands.parallel(climb.setPosition(ClimbConstants.ready)));
    // Auto Drive to Closest Cage
    stateTriggers
        .get(RobotState.ClimbReady)
        .and(driver.back())
        .onTrue(autoAlign.driveToCage(drive));
    stateTriggers
        .get(RobotState.ClimbStow)
        .onTrue(
            Commands.parallel(climb.setPosition(ClimbConstants.stow), forceState(RobotState.Idle)));

    /*
     * RobotCancelRequests -- Combination of Left or Right Triggers
     */
    // Robot is in Intake and in order to change to Idle - Driver hits A
    stateTriggers.get(RobotState.Intake).and(driver.povLeft()).onTrue(forceState(RobotState.Idle));

    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(driver.povLeft())
        .onTrue(forceState(RobotState.Idle));

    // Robot is Auto Aligning for Algae, Cancel Pre_Algae - Driver hits LT
    stateTriggers
        .get(RobotState.PreAlgae)
        .and(driver.povLeft())
        .onTrue(forceState(RobotState.Idle));

    // Elevator is at Algae Intake Height, Cancel Request is RT - Driver
    stateTriggers
        .get(RobotState.AlgaeSetpoint)
        .and(driver.povLeft())
        .onTrue(forceState(RobotState.Idle));

    // Robot is Intaking Algae using LT, Cancel Request is RT
    stateTriggers
        .get(RobotState.AlgaeIntake)
        .and(driver.povLeft())
        .onTrue(forceState(RobotState.Idle));

    // Robot has a Algae, Cancel Request is LT, to Drop Algae, and go back to Idle
    stateTriggers
        .get(RobotState.AlgaeArmed)
        .and(driver.povLeft())
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
  }
}
