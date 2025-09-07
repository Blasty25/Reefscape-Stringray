package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbIOSim implements ClimbIO {
  // this is stolen from the shoulder but its like fine our physics sim isnt really good enough for
  // full climb sim
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          ClimbConstants.gearing,
          0.52,
          Units.inchesToMeters(13.5),
          0.0,
          Units.degreesToRadians(270.0),
          true,
          0.0);

  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(10.0, 10.0));

  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(ClimbIOInputsAutoLogged inputs) {
    armSim.update(0.02);

    inputs.motorVelocityRadPerSec =
        RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.motorPositionRad = armSim.getAngleRads();
    inputs.motorStatorCurrentAmps = armSim.getCurrentDrawAmps();
    inputs.motorSupplyCurrentAmps = 0.0;
    inputs.motorTempCelsius = 0.0;
    inputs.motorAppliedVolts = appliedVoltage;
  }

  @Override
  public void setVoltage(final double voltage) {
    appliedVoltage = voltage;
    armSim.setInputVoltage(voltage);
  }

  @Override
  public void setPosition(final Rotation2d position) {
    setVoltage(
        pid.calculate(armSim.getAngleRads(), position.getRadians())
            + feedforward.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity));
  }

  @Override
  public void resetEncoder(double position) {
    armSim.setState(position, 0);
  }
}
