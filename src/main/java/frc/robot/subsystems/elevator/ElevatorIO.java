// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double targetHeight = 0.0;
    public double position = 0.0;

    public double leftVolts = 0.0;
    public double rightVolts = 0.0;

    public double leftStatorCurrent = 0.0;
    public double rightStatorCurrent = 0.0;

    public double leftSupplyCurrent = 0.0;
    public double rightSupplyCurrent = 0.0;

    public double leftTemp = 0.0;
    public double rightTemp = 0.0;

    public double velocity = 0.0;

    public boolean leftConnected = false;
    public boolean rightConnected = false;

    public boolean atSetpoint = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVolts(double volts) {}

  public default void setControl(double position) {}

  public default void resetEncoder() {}

  public default double positionError() {
    return 0.0;
  }
}
