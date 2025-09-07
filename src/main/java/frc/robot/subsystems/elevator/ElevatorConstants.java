// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {

  // sim stuff
  public static final double elevatorMass =
      Pounds.of(30.0).in(Kilograms); // Returns The mass in kilograms
  public static final double maxHeight = Inches.of(69.6).in(Meters);
  public static final double minHeight = Inches.of(0).in(Meters);
  public static final boolean simGravity = false;
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

  public enum ElevatorSetpoints {
    INTAKE,
    L1,
    L2,
    L3,
    L4,
    A2,
    A3
  }

  // TARGET SETPOINTS in INCHES!!!
  public static final double L4Setpoint = Meters.fromBaseUnits(1.77);
  public static final double L3Setpoint = Meters.fromBaseUnits(1.18);
  public static final double L2Setpoint = Meters.fromBaseUnits(0.79);
  public static final double L1Setpoint = Meters.fromBaseUnits(0.42);
  public static final double IntakeSetpoint = Meters.fromBaseUnits(0.014);
  public static final double A2Setpoint = Meters.fromBaseUnits(0.42);
  public static final double A3Setpoint = Meters.fromBaseUnits(0.81);

  public static LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/PID/kP", 45.0);
  public static LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/PID/kI", 0.0);
  public static LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/PID/kD", 1.0);

  public static LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/FF/kS", 0.0);
  public static LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/FF/kG", 0.0);
  public static LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/FF/kV", 4.8);
  public static LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/FF/kA", 5.0);

  // Real Robot numbers
  public static double rampRate = 0.1;
  public static double maxVelocity = 3.5;
  public static double maxAcceleration = 10.0;
  public static int statorCurrent = 100;
  public static int supplyCurrent = 80;
  public static int supplyCurrentLow = 60;

  public static final double drumRadius = 5.0 / 1000.0 * 36 / (2.0 * Math.PI);
  public static final double gearing = (5.0 / 1.0);
  public static final double positionConversionFactor = drumRadius * 2 * Math.PI / gearing;
  public static final double tolerance = .005;
}
