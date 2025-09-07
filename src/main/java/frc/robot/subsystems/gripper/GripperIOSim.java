package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class GripperIOSim implements GripperIO {
  private final DCMotorSim motorSim;

  private double appliedVolts = 0.0;
  private LoggedNetworkBoolean isAlgaeDetected =
      new LoggedNetworkBoolean("/Tuning/SimState/AlgaeDetected", false);

  public GripperIOSim() {
    this.motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), GripperConstants.moi, GripperConstants.gearing),
            DCMotor.getKrakenX60(1));
    isAlgaeDetected.set(false);
  }

  @Override
  public void updateInputs(GripperIOInputsAutoLogged inputs) {
    motorSim.update(0.02);
    inputs.motorVelocityRPM = motorSim.getAngularVelocityRPM();
    inputs.motorAppliedVolts = motorSim.getInputVoltage();
    inputs.motorCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.motorTempCelsius = 0.0;
    inputs.hasAlgae = isAlgaeDetected.get();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    motorSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void setDetected(boolean detected) {
    isAlgaeDetected.set(detected);
  }
}
