// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandlerConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {

  private OuttakeIOInputsAutoLogged inputs;
  private OuttakeIO io;

  public Outtake(OuttakeIO io) {
    this.io = io;
    inputs = new OuttakeIOInputsAutoLogged();
  }

  public boolean isDetected() {
    return inputs.isCorralDetected;
  }

  public void stop() {
    io.stop();
  }

  public void setPercentOutput(double output) {
    io.setPercent(output);
  }

  public Command ejectCorral() {
    return Commands.run(
            () -> {
              io.eject(-6);
              io.setSimState(false);
            },
            this)
        .until(() -> !inputs.isCorralDetected);
  }

  public Command shoot() {
    return Commands.run(
            () -> {
              io.setVoltage(5.0);
              io.setSimState(false);
            },
            this)
        .until(() -> !inputs.isCorralDetected)
        .finallyDo(() -> io.setVoltage(5.0));
  }

  public Command setSimState(boolean state) {
    return Commands.runOnce(() -> io.setSimState(state));
  }

  // Intake corral with 12 volts and stop once its detected!
  public Command intake() {
    return Commands.run(
            () -> {
              io.intake();
            },
            this)
        .until(() -> inputs.isCorralDetected)
        .finallyDo(() -> StateHandlerConstants.rumble(0.8, 0.5).schedule());
  }

  public Command overideShoot(DoubleSupplier forward, DoubleSupplier backward) {
    return Commands.runEnd(
        () -> io.setVoltage((forward.getAsDouble() - backward.getAsDouble()) * 12),
        () -> io.setVoltage(0.0),
        this);
  }

  public Command overideShoot(double steadyVolts) {
    return Commands.runEnd(() -> io.setVoltage(steadyVolts), () -> io.setVoltage(0.0), this);
  }

  public void runVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Outtake", inputs);
    io.updateInputs(inputs);
  }
}
