// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private boolean isHomed = false;
  private Timer homingTimer = new Timer();
  private Debouncer homingDebouncer = new Debouncer(0.5);

  @AutoLogOutput(key = "/Elevator/Setpoint")
  public static ElevatorSetpoints setpoint = ElevatorSetpoints.INTAKE;

  @AutoLogOutput(key = "/Elevator/PreviousSetpoint")
  private ElevatorSetpoints previousSetpoints = ElevatorSetpoints.INTAKE;

  private Distance difference = Meters.zero();

  public Elevator(ElevatorIO io) {
    this.io = io;
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
              isHomed =
                  homingDebouncer.calculate(Math.abs(inputs.velocity.in(MetersPerSecond)) <= 0.1);
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

  public ElevatorSetpoints getSetpoint() {
    return setpoint;
  }

  @Override
  public void periodic() {
    Logger.processInputs("Elevator", inputs);
    io.updateInputs(inputs);
    difference = (inputs.targetHeight.minus(inputs.position));
    Logger.recordOutput("/Elevator/Difference", difference.in(Meters));
    Logger.recordOutput("Elevator/TargetHeight", inputs.targetHeight);

    if (setpoint.equals(ElevatorSetpoints.INTAKE)) {
      setPosition(IntakeSetpoint, 0);
    }
    if (setpoint.equals(ElevatorSetpoints.L1)) {
      setPosition(L1Setpoint, 0);
    }
    if (setpoint.equals(ElevatorSetpoints.L2)) {
      setPosition(L2Setpoint, 0);
    }
    if (setpoint.equals(ElevatorSetpoints.L3)) {
      setPosition(L3Setpoint, 0);
    }
    if (setpoint.equals(ElevatorSetpoints.L4)) {
      setPosition(L4Setpoint, 0);
    }
    if (setpoint.equals(ElevatorSetpoints.A2)) {
      setPosition(A2Setpoint, 0);
    }
    if (setpoint.equals(ElevatorSetpoints.A3)) {
      setPosition(A3Setpoint, 0);
    }
  }
}
