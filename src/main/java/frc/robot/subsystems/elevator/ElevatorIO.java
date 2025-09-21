// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public Distance targetHeight = Meters.zero();
    public Distance position = Meters.zero();

    public Voltage leftVolts = Volts.zero();
    public Voltage rightVolts = Volts.zero();

    public Current leftCurrent = Amps.zero();
    public Current rightCurrent = Amps.zero();

    public double leftTemp = 0.0;
    public double rightTemp = 0.0;

    public LinearVelocity velocity = MetersPerSecond.zero();

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
