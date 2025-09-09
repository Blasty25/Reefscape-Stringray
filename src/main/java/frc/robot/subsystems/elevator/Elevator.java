// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private boolean isHomed = false;
  private Timer homingTimer = new Timer();
  private Debouncer homingDebouncer = new Debouncer(0.5);
  private SysIdRoutine routine;
  @AutoLogOutput(key = "/Elevator/Setpoint")
  public static ElevatorSetpoints setpoint = ElevatorSetpoints.INTAKE;

  @AutoLogOutput(key = "/Elevator/PreviousSetpoint")
  private ElevatorSetpoints previousSetpoints = ElevatorSetpoints.INTAKE;

  public Elevator(ElevatorIO io) {
    this.io = io;
    setpointMap.put(ElevatorSetpoints.INTAKE, IntakeSetpoint);
    setpointMap.put(ElevatorSetpoints.L1, L1Setpoint);
    setpointMap.put(ElevatorSetpoints.L2, L2Setpoint);
    setpointMap.put(ElevatorSetpoints.L3, L3Setpoint);
    setpointMap.put(ElevatorSetpoints.L4, L4Setpoint);
    setpointMap.put(ElevatorSetpoints.A2, A2Setpoint);
    setpointMap.put(ElevatorSetpoints.A3, A3Setpoint);
    
    routine = new SysIdRoutine(
        new Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Elevator/SysIdTestStateVolts", state.toString())),
        new Mechanism((volts) -> io.setVolts(volts.in(Volts)),
            log -> {
              log.motor("left")
                  .voltage(inputs.leftVolts)
                  .current(inputs.leftCurrent)
                  .linearPosition(inputs.position)
                  .linearVelocity(inputs.velocity);
              log.motor("right")
                  .voltage(inputs.rightVolts)
                  .current(inputs.rightCurrent);
            }, this));
  }

  public void setVoltage(double volts) {
    io.setVolts(volts);
  }

  public void setPosition(double position, double velocity) {
    io.setControl(position, velocity);
  }

  public Command resetEncoder() {
    return Commands.runOnce(
        () -> {
          io.resetEncoder();
        },
        this)
        .ignoringDisable(true);
  }

  public Command setTarget(ElevatorSetpoints nextSetpoints) {
    return Commands.runOnce(
        () -> {
          System.out.println("Changing Setpoint from " + previousSetpoints + " to " + setpoint);
          previousSetpoints = setpoint;
          setpoint = nextSetpoints;
        });
  }

  public Command homeElevator() {
    return Commands.startRun(
        () -> {
          isHomed = false;
          homingTimer.restart();
          homingDebouncer.calculate(false);
        },
        () -> {
          io.setVolts(-4);
          isHomed = homingDebouncer.calculate(Math.abs(inputs.velocity.in(MetersPerSecond)) <= 0.1);
        },
        this)
        .until(() -> isHomed)
        .finallyDo(
            () -> {
              System.out.println("Elevator is Homed in " + homingTimer.get());
              io.setVolts(0.0);
              io.resetEncoder();
              System.out.println(
                  "Elevator was homed and encoder was reseted in " + homingTimer.get());
            });
  }

  /*Should be able to retune elevator and get better constnats REMEMBER TO SET PID TO 0*/
  public Command sysId() {
    return Commands.sequence(
        routine.dynamic(Direction.kForward),
        routine.dynamic(Direction.kReverse),
        routine.quasistatic(Direction.kForward),
        routine.quasistatic(Direction.kReverse));
  }

  public ElevatorSetpoints getSetpoint() {
    return setpoint;
  }

  @Override
  public void periodic() {
    Logger.processInputs("Elevator", inputs);
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator/TargetHeight", inputs.targetHeight);

    setPosition(setpointMap.get(getSetpoint()), 0.0);
  }
}
