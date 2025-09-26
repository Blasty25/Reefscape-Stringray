// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
  @AutoLog
  public class OuttakeIOInputs {

    public double motorVoltage = 0.0;

    public double statorCurrent = 0.0;

    public double velocityRadPerSec = 0.0;

    public double rawMeasurement = 0.0;

    public boolean motorConnected = false;
    public boolean canAndColorConnected = false;

    public boolean motorStalled = false;

    public boolean isCorralDetected = false;
  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void setPercent(double percent) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double radPerSec) {}

  public default void eject(double eject) {}

  public default void intake() {}

  public default void stop() {}

  public default void setSimState(boolean detected) {}
}
