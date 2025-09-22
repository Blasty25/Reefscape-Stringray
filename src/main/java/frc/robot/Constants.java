// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum RobotState {
    Idle,
    Intake,
    SetElevatorSetpoint,
    Manual_Elevator,
    Manual_Score,
    Shoot,
    RevFunnel,
    Eject,
    PreAlgae,
    AlgaeSetpoint,
    AlgaeIntake,
    AlgaeArmed,
    ClimbReady,
    ClimbPull,
    ClimbStow
  }

  public static void logMotorStatus(String name, boolean deboundedMotor) {
    SmartDashboard.putBoolean("Motors/State" + name, deboundedMotor);
  }

  public static boolean getMotorStatus(String name) {
    return SmartDashboard.getBoolean("Motors/State" + name, false);
  }
}
