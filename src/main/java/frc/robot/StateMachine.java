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
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.outtake.Outtake;
import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;

public class StateMachine extends SubsystemBase {

  private Drive drive;
  private Elevator elevator;
  private Outtake outtake;
  private CommandXboxController driver;
  private AutoAlign autoAlign;
  private Gripper gripper;
  private Climb climb;
  private LED led;

  private Map<RobotState, Trigger> stateRequests = new EnumMap<>(RobotState.class);
  private Map<RobotState, Trigger> stateTriggers = new EnumMap<>(RobotState.class);

  @AutoLogOutput(key = "RobotState/CurrentState")
  private RobotState state = RobotState.Idle;

  @AutoLogOutput(key = "RobotState/PreviousState")
  private RobotState previousState = RobotState.Idle;

  public StateMachine(
      Drive drive,
      Elevator elevator,
      Outtake outtake,
      Gripper gripper,
      Climb climb,
      AutoAlign autoAlign,
      LED led,
      CommandXboxController controller) {

    this.drive = drive;
    this.elevator = elevator;
    this.outtake = outtake;
    this.driver = controller;
    this.autoAlign = autoAlign;
    this.gripper = gripper;
    this.climb = climb;
    this.led = led;

    stateRequests.put(RobotState.Idle, driver.a());
    stateRequests.put(RobotState.SetElevatorSetpoint, driver.povRight());
    stateRequests.put(RobotState.Manual_Score, driver.povLeft());
    stateRequests.put(RobotState.Shoot, driver.rightTrigger());
    stateRequests.put(RobotState.Eject, driver.leftTrigger());
    stateRequests.put(RobotState.Climb_Ready, driver.leftTrigger());
    stateRequests.put(RobotState.Climb_Stow, driver.povDown());
    stateRequests.put(RobotState.Climb_Pull, driver.rightTrigger());

    for (RobotState state : RobotState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
    }

    enableStateSetup();
  }

  public void enableStateSetup() {

    // Boolean Checking State Commands
    stateTriggers
        .get(RobotState.Idle)
        .and(() -> outtake.isDetected())
        .onTrue(
            Commands.parallel(
                forceState(RobotState.SetElevatorSetpoint),
                led.setState(RobotState.SetElevatorSetpoint)));
    stateTriggers
        .get(RobotState.Idle)
        .and(() -> !gripper.getDualDetected())
        .onTrue(gripper.setVoltage(0.0));

    stateTriggers
        .get(RobotState.Shoot)
        .and(() -> !outtake.isDetected())
        .onTrue(forceState(RobotState.Idle));
    stateTriggers
        .get(RobotState.Idle)
        .and(driver.povRight())
        .onTrue(forceState(RobotState.Manual_Score));
    stateTriggers
        .get(RobotState.Idle)
        .and(driver.leftBumper())
        .and(driver.rightBumper())
        .onTrue(forceState(RobotState.Pre_Algae));
    stateTriggers
        .get(RobotState.Algae_Intake)
        .and(() -> gripper.getDualDetected())
        .onTrue(forceState(RobotState.Algae_Armed));
    stateTriggers
        .get(RobotState.Idle)
        .and(stateRequests.get(RobotState.Climb_Ready))
        .onTrue(
            Commands.parallel(
                forceState(RobotState.Climb_Ready), led.setState(RobotState.Climb_Ready)));
    stateTriggers
        .get(RobotState.Climb_Ready)
        .and(stateRequests.get(RobotState.Climb_Stow))
        .onTrue(forceState(RobotState.Climb_Stow));
    stateTriggers
        .get(RobotState.Climb_Ready)
        .and(stateRequests.get(RobotState.Climb_Pull))
        .onTrue(
            Commands.parallel(
                forceState(RobotState.Climb_Pull), led.setState(RobotState.Climb_Pull)));
    stateTriggers.get(RobotState.Idle).onTrue(led.setState(RobotState.Idle));

    // State Trigger Commands

    /* Reset gyro command */
    stateTriggers
        .get(RobotState.Idle)
        .and(driver.b())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    /* Stop drive with a X formation */
    stateTriggers
        .get(RobotState.Idle)
        .and(driver.x())
        .onTrue(Commands.runOnce(drive::stopWithX, drive));

    /*
     * Allow the Corral to be ejected if corral is in the Carriage
     * should be used if the Corral is in the carriage at the wrong angle
     * Control is Left Trigger
     */
    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(stateRequests.get(RobotState.Eject))
        .onTrue(
            Commands.sequence(
                led.setState(RobotState.Eject),
                outtake.ejectCorral().andThen(forceState(RobotState.Idle))));

    /*
     * The following 4 stateTriggers are elevator setpoints
     * once one of the following, a, x, b or y is pressed,
     * it will log that setpoint and the elevator will move once
     * driver is ready.
     * a = L1
     * x = L2
     * b = L3
     * y = L4
     */

    stateTriggers.get(RobotState.Idle).onTrue(elevator.setTarget(ElevatorSetpoints.INTAKE));

    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(driver.a())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.L1),
                forceState(RobotState.Shoot),
                led.setState(RobotState.Shoot)));

    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(driver.x())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.L2),
                forceState(RobotState.Shoot),
                led.setState(RobotState.Shoot)));

    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(driver.b())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.L3),
                forceState(RobotState.Shoot),
                led.setState(RobotState.Shoot)));

    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(driver.y())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.L4),
                forceState(RobotState.Shoot),
                led.setState(RobotState.Shoot)));

    /* Change elevator in Shoot States */
    stateTriggers
        .get(RobotState.Shoot)
        .and(driver.a())
        .onTrue(elevator.setTarget(ElevatorSetpoints.L1));

    stateTriggers
        .get(RobotState.Shoot)
        .and(driver.x())
        .onTrue(elevator.setTarget(ElevatorSetpoints.L2));

    stateTriggers
        .get(RobotState.Shoot)
        .and(driver.b())
        .onTrue(elevator.setTarget(ElevatorSetpoints.L3));

    stateTriggers
        .get(RobotState.Shoot)
        .and(driver.y())
        .onTrue(elevator.setTarget(ElevatorSetpoints.L4));

    /*
     * Align to Reef Peg during the set Elevator State, e.g you can align
     * while elevator is down
     */
    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(driver.leftBumper())
        .onTrue(autoAlign.driveToAlignWithReef(drive, true, elevator.getSetpoint()));

    stateTriggers
        .get(RobotState.SetElevatorSetpoint)
        .and(driver.rightBumper())
        .onTrue(autoAlign.driveToAlignWithReef(drive, false, elevator.getSetpoint()));

    /*
     * Align to Reef pegs during the set Shoot State, e.g you can align
     * while elevator is up, minor adjustments
     */
    stateTriggers
        .get(RobotState.Shoot)
        .and(driver.leftBumper())
        .onTrue(autoAlign.driveToAlignWithReef(drive, true, elevator.getSetpoint()));

    stateTriggers
        .get(RobotState.Shoot)
        .and(driver.rightBumper())
        .onTrue(autoAlign.driveToAlignWithReef(drive, false, elevator.getSetpoint()));

    /* Align to the middle of the too Reef Pegs in order to pick up Algae */
    stateTriggers
        .get(RobotState.Pre_Algae)
        .onTrue(
            Commands.parallel(
                autoAlign.driveToAlgaePose(drive),
                forceState(RobotState.Algae_Setpoint),
                led.setState(RobotState.Pre_Algae)));

    /* If Robot is in Pre Algae then driver has the option to set algae setpoint */
    stateTriggers // Hit x to get to Algae on L3
        .get(RobotState.Algae_Setpoint)
        .and(driver.x())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.A3),
                forceState(RobotState.Algae_Intake),
                led.setState(RobotState.Pre_Algae)));
    stateTriggers // Hit b to get to Algae on L2
        .get(RobotState.Algae_Setpoint)
        .and(driver.b())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.A2),
                forceState(RobotState.Algae_Intake),
                led.setState(RobotState.Pre_Algae)));

    /* Intake Algae into the Gripper */
    stateTriggers
        .get(RobotState.Algae_Intake)
        .and(driver.leftTrigger())
        .onTrue(gripper.setVoltage(GripperConstants.A23));

    /* Once Algae is intaked set Elevator to Intake in order to move */
    stateTriggers
        .get(RobotState.Algae_Armed)
        .onTrue((elevator.setTarget(ElevatorSetpoints.INTAKE)));

    /* Shoot the Algae using Right Trigger either at the Processer or Barge */
    stateTriggers
        .get(RobotState.Algae_Armed)
        .and(driver.rightTrigger())
        .onTrue(
            Commands.parallel(
                gripper.setVoltage(GripperConstants.AP),
                Commands.runOnce(() -> gripper.setSimDetected(false)),
                Commands.runOnce(() -> System.out.println("Algae Shot")),
                elevator.setTarget(ElevatorSetpoints.INTAKE),
                forceState(RobotState.Idle)));

    /* Set the Elevator height to the Barge to shoot */
    stateTriggers
        .get(RobotState.Algae_Armed)
        .and(driver.y())
        .onTrue(elevator.setTarget(ElevatorSetpoints.L4));

    /* Shoot the Coral, and set the Elevator Back down to intake */
    stateTriggers
        .get(RobotState.Shoot)
        .and(driver.rightTrigger())
        .onTrue(
            Commands.sequence(
                outtake.shoot(),
                elevator.setTarget(ElevatorSetpoints.INTAKE),
                forceState(RobotState.Idle),
                led.setState(RobotState.Idle)));

    /*
     * Manual Scoring States, be ableto set the elevator to these states, without
     * the need of having a Coral
     */
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(driver.y())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.L4), led.setState(RobotState.Manual_Score)));
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(driver.x())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.L3), led.setState(RobotState.Manual_Score)));
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(driver.b())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.L2), led.setState(RobotState.Manual_Score)));
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(driver.a())
        .onTrue(
            Commands.parallel(
                elevator.setTarget(ElevatorSetpoints.L1), led.setState(RobotState.Manual_Score)));

    /* Shoot the Coral during Manual Score State, then set it back to Idle */
    stateTriggers
        .get(RobotState.Manual_Score)
        .and(driver.rightTrigger())
        .onTrue(outtake.shoot().andThen(forceState(RobotState.Idle)));

    // ENDGAME!
    /* Commands to set Climb States, one for pull, one for stow, one for ready */
    stateTriggers
        .get(RobotState.Climb_Pull)
        .onTrue(
            Commands.parallel(
                climb.setPosition(ClimbConstants.climbed), led.setState(RobotState.Climb_Pull)));
    stateTriggers
        .get(RobotState.Climb_Ready)
        .onTrue(
            Commands.parallel(
                climb.setPosition(ClimbConstants.ready), led.setState(RobotState.Climb_Ready)));
    stateTriggers
        .get(RobotState.Climb_Stow)
        .onTrue(
            Commands.parallel(climb.setPosition(ClimbConstants.stow), forceState(RobotState.Idle)));
  }

  private Command forceState(RobotState nextState) {
    return Commands.runOnce(
        () -> {
          System.out.println("Changing state to " + nextState);

          previousState = state;
          state = nextState;
        });
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      forceState(RobotState.Idle).schedule();
    }

    // Experimental rumble design!
    if (Timer.getMatchTime() <= 15.0 && DriverStation.isTeleopEnabled()) {
      controller.setRumble(RumbleType.kBothRumble, 0.5);
    }
    if (Timer.getMatchTime() >= 10.0 && DriverStation.isTeleopEnabled()) {
      controller.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }
}
