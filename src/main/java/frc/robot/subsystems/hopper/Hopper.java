// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandlerConstants;
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

  public Command autoIntake(Outtake outtake, double hopperVolts, double outtakeIntakeVolts) {
    return Commands.run(
            () -> {
              outtake.setOuttakeVolts(outtakeIntakeVolts);
              this.setVoltage(hopperVolts);
            },
            this)
        .until(() -> outtake.isDetected())
        .finallyDo(
            () -> {
              StateHandlerConstants.rumble(0.8, 0.5).schedule();
              this.setVoltage(0.0);
              outtake.stop();
            });
  }

  public Command eject() {
    return Commands.run(
            () -> {
              this.setVoltage(-12.0);
            },
            this)
        .finallyDo(() -> this.setVoltage(0.0));
  }

  public Command intake(Outtake outtake, double intakeVolts) {
    return Commands.run(
            () -> {
              this.setVoltage(intakeVolts);
            },
            this)
        .until(() -> outtake.isDetected())
        .finallyDo(() -> io.stop());
  }

  @Override
  public void periodic() {
    Logger.processInputs("Hopper", inputs);
    io.updateInputs(inputs);
  }
}
