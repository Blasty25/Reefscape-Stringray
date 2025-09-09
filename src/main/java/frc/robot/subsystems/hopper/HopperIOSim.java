package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.outtake.OuttakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
  private DCMotorSim hopper;

  public HopperIOSim() {
    hopper =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), moi, gearing),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopper.update(0.02);
    inputs.motorCurrent = hopper.getCurrentDrawAmps();
    inputs.motorTemp = 20.0;
    inputs.motorVelocity = hopper.getAngularVelocityRadPerSec();
    inputs.motorVolts = hopper.getInputVoltage();
  }

  @Override
  public void stop() {
    hopper.setInputVoltage(0.0);
  }

  @Override
  public void setTrackPercent(double percent) {
    hopper.setInputVoltage(percent / 12);
  }

  @Override
  public void eject(double ejectValue) {
    hopper.setInputVoltage(ejectValue);
  }

  @Override
  public void setVelocity(double radPerSec) {
    hopper.setAngularVelocity(radPerSec);
  }
}
