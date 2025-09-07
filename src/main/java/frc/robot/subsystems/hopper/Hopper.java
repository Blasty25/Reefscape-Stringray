// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    Logger.processInputs("Hopper", inputs);
    io.updateInputs(inputs);
  }
}
