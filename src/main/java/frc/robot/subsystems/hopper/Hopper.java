// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.outtake.Outtake;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs;

  public Hopper(HopperIO io) {
    this.io = io;
    inputs = new HopperIOInputsAutoLogged();
  }

  public void eject(double ejectValue) {
    io.eject(ejectValue);
  }

  public void setTrackPercent(double percent) {
    io.setTrackPercent(percent);
  }

  public void setTrackVelocity(double radPerSec) {
    io.setVelocity(radPerSec);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public Command overideVoltage(double volts) {
    return Commands.runEnd(() -> setVoltage(volts), () -> setVoltage(0.0), this);
  }

  public Command stopHopper() {
    return Commands.runOnce(() -> io.stop());
  }

  public Command autoIntake(Outtake outtake) {
    return Commands.run(
            () -> {
              this.setTrackPercent(1.0);
              outtake.intake();
            },
            this)
        .until(() -> outtake.isDetected());
  }

  public Command eject() {
    return Commands.run(
        () -> {
          this.setTrackPercent(-1.0);
        },
        this);
  }

  public Command intake() {
    return Commands.run(
        () -> {
          this.setVoltage(3.0);
        },
        this);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Hopper", inputs);
    io.updateInputs(inputs);
  }
}
