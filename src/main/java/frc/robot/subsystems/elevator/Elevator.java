// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private boolean isHomed = false;
  private Timer homingTimer = new Timer();
  private Debouncer homingDebouncer = new Debouncer(0.5);
  private SysIdRoutine routine;

  @AutoLogOutput(key = "Elevator/Setpoint")
  private ElevatorSetpoint setpoint = ElevatorSetpoint.INTAKE;

  @AutoLogOutput(key = "Elevator/NextSetpoint")
  private ElevatorSetpoint nextSetpoint = ElevatorSetpoint.INTAKE;

  public Elevator(ElevatorIO io) {
    this.io = io;

    routine =
        new SysIdRoutine(
            new Config(null, Volts.of(4), null),
            new Mechanism(
                (volts) -> io.setVolts(volts.in(Volts)),
                log -> {
                  log.motor("left")
                      .voltage(Volts.of(inputs.leftVolts))
                      .current(Amps.of(inputs.leftSupplyCurrent))
                      .linearPosition(Meters.of(inputs.position))
                      .linearVelocity(MetersPerSecond.of(inputs.velocity));
                },
                this));
  }

  public void setVoltage(double volts) {
    io.setVolts(volts);
  }

  public Command overideElevator(DoubleSupplier volts) {
    return Commands.run(
        () -> {
          io.setVolts(volts.getAsDouble() * 12);
        },
        this);
  }

  /* Set Position to ... */
  public void setPosition(double position) {
    io.setControl(position);
  }

  /* Reset Encoder with a position value of 0 */
  public Command resetEncoder() {
    return Commands.runOnce(
            () -> {
              io.resetEncoder();
            },
            this)
        .ignoringDisable(true);
  }

  public void selectFutureTarget(ElevatorSetpoint setpoint) {
    nextSetpoint = setpoint;
  }

  /* Set the Elevator Target enum, for set extension method to move the elevator */
  public Command setTarget(ElevatorSetpoint height) {
    return Commands.runOnce(() -> this.selectFutureTarget(height));
  }

  public Command setExtension() {
    return Commands.runOnce(() -> setpoint = nextSetpoint);
  }

  public Command homeElevator() {
    return Commands.startRun(
            () -> {
              isHomed = false;
              homingTimer.restart();
              homingDebouncer.calculate(false);
            },
            () -> {
              io.setVolts(-6);
              isHomed = homingDebouncer.calculate(Math.abs(inputs.velocity) <= 0.1);
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

  /*
   * Should be able to retune elevator and get better constnats REMEMBER TO SET
   * PID TO 0
   */
  public Command sysId() {
    return Commands.sequence(
        routine.dynamic(Direction.kForward),
        routine.dynamic(Direction.kReverse),
        routine.quasistatic(Direction.kForward),
        routine.quasistatic(Direction.kReverse));
  }

  public ElevatorSetpoint getSetpoint() {
    return setpoint;
  }

  public ElevatorSetpoint getNextExpectedSetpoint() {
    return nextSetpoint;
  }

  public boolean atSetpoint() {
    return inputs.atSetpoint;
  }

  @Override
  public void periodic() {
    Logger.processInputs("Elevator", inputs);
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator/TargetHeight", inputs.targetHeight);
    inputs.atSetpoint = Math.abs(inputs.targetHeight - inputs.position) <= tolerance;
    this.setPosition(getSetpoint().height);
  }
}
