package frc.robot.subsystems.outtake;

import static frc.robot.subsystems.outtake.OuttakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class OuttakeIOSim implements OuttakeIO {
  private DCMotorSim carriage;
  private LoggedNetworkBoolean isDetectedInSim =
      new LoggedNetworkBoolean("/Tuning/SimState/CorralDetected", false);

  public OuttakeIOSim() {
    carriage =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), moi, gearing),
            DCMotor.getKrakenX60(1));
    isDetectedInSim.set(false);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {

    carriage.update(0.02);

    inputs.isCorralDetected = isDetectedInSim.get();

    inputs.motorConnected = true;

    inputs.statorCurrent = carriage.getCurrentDrawAmps();

    inputs.motorVoltage = carriage.getInputVoltage();

    inputs.velocityRadPerSec = carriage.getAngularVelocityRadPerSec();
  }

  @Override
  public void stop() {
    carriage.setInputVoltage(0.0);
  }

  @Override
  public void setPercent(double percent) {
    carriage.setInputVoltage(percent * RobotController.getBatteryVoltage()); // Percent Output
    Commands.waitSeconds(1.5);
    isDetectedInSim.set(false);
  }

  @Override
  public void setVelocity(double radPerSec) {
    carriage.setAngularVelocity(radPerSec);
  }

  // Same thing with intake just to make it work in sim
  @Override
  public void eject(double eject) {
    if (isDetectedInSim.get()) {
      this.setVoltage(eject);
      isDetectedInSim.set(false);
    }
  }

  /*
   * This is a makeshift logic for sim on how it will intake a corral, run intake
   * until corral is detected, for sim it is set to run intake for 4 seconds then
   * set isDetected to true
   */
  @Override
  public void intake() {
    if (isDetectedInSim.get()) {
      carriage.setInputVoltage(0);
      return;
    } else {
      Logger.recordOutput("/Outtake/Intaking", true);
      carriage.setInputVoltage(6);
      return;
    }
  }

  @Override
  public void setVoltage(double volts) {
    carriage.setInputVoltage(volts);
  }

  @Override
  public void setSimState(boolean detected) {
    isDetectedInSim.set(detected);
  }
}
