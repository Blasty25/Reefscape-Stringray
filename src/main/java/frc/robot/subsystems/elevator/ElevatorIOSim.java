// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim elevator;

  private ProfiledPIDController pid =
      new ProfiledPIDController(
          kP.getAsDouble(), 0.0, 0.0, new TrapezoidProfile.Constraints(3.8, 4));

  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kG.getAsDouble());
  private double volts = 0.0;

  public ElevatorIOSim() {
    this.pid.setTolerance(0.019);
    this.elevator =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(gearbox, elevatorMass, drumRadius, gearing),
            gearbox,
            minHeight,
            maxHeight,
            simGravity,
            minHeight);
    ff.maxAchievableVelocity(12, elevator.getVelocityMetersPerSecond());
    ff.maxAchievableAcceleration(12, elevator.getVelocityMetersPerSecond());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevator.update(0.02);
    inputs.leftVolts = Volts.of(volts);
    inputs.position = Meters.of(elevator.getPositionMeters());
    inputs.velocity = MetersPerSecond.of(elevator.getVelocityMetersPerSecond());
    inputs.leftVolts = Volts.of(volts);
    inputs.rightVolts = Volts.of(volts);

    inputs.leftCurrent = Amps.of(elevator.getCurrentDrawAmps());
    inputs.rightCurrent = Amps.of(elevator.getCurrentDrawAmps());

    inputs.targetHeight = Meters.of(pid.getGoal().position);

    if (kP.hasChanged(hashCode())) {
      pid.setP(kP.getAsDouble());
    }

    if (kI.hasChanged(hashCode())) {
      pid.setI(kI.getAsDouble());
    }

    if (kD.hasChanged(hashCode())) {
      pid.setD(kD.getAsDouble());
    }

    if (kV.hasChanged(hashCode())) {
      ff.setKv(kV.getAsDouble());
    }

    if (kS.hasChanged(hashCode())) {
      ff.setKs(kS.getAsDouble());
    }

    if (kA.hasChanged(hashCode())) {
      ff.setKa(kA.getAsDouble());
    }

    if (kG.hasChanged(hashCode())) {
      ff.setKg(kG.getAsDouble());
    }
  }

  @Override
  public void setControl(double position, double velocity) {
    pid.setGoal(position);
    setVolts((pid.calculate(elevator.getPositionMeters())));
  }

  @Override
  public void setVolts(double voltage) {
    this.volts = voltage;
    elevator.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void resetEncoder() {
    elevator.setInputVoltage(0);
  }

  @Override
  public double positionError() {
    Logger.recordOutput("Debug/Elevator/Error", pid.getPositionError());
    return pid.getPositionError();
  }
}
