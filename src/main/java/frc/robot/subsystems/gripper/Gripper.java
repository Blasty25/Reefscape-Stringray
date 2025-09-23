package frc.robot.subsystems.gripper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  private final Debouncer currentDebouncer = new Debouncer(0.25);
  private final Debouncer dualDebouncer = new Debouncer(0.25);

  public Gripper(GripperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Gripper", inputs);
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
        () -> {
          io.setVoltage(volts.getAsDouble());
        });
  }

  public Command setVoltage(double voltage) {
    io.setDetected(false);
    return this.setVoltage(() -> voltage);
  }

  public double getVoltage() {
    return inputs.motorAppliedVolts;
  }

  public boolean getCurrentDetected() {
    return currentDebouncer.calculate(
        (inputs.motorCurrentAmps > 15) && (Math.abs(inputs.motorVelocityRPM) < 5));
  }

  public boolean getDetected() {
    return inputs.hasAlgae;
  }

  public boolean getDualDetected() {
    boolean debounced = dualDebouncer.calculate(getCurrentDetected() || getDetected());
    Logger.recordOutput("Gripper/Detected", debounced);
    return debounced;
  }

  public boolean isRobotInFrontOfBarge(Pose2d robot) {
    Pose2d barge = new Pose2d(8.67, 6.18, Rotation2d.fromDegrees(-180.0));
    Transform2d rel = new Transform2d(barge, robot);
    Logger.recordOutput("/LED/AutoBargePose", barge.relativeTo(robot));
    double forwardRange = 1.5; // meters in front
    double sideRange = 0.81; // half barge width + margin

    return (rel.getX() > 0)
        && // in front
        (rel.getX() <= forwardRange)
        && // within distance
        (Math.abs(rel.getY()) <= sideRange); // not too far sideways
  }

  public void setSimDetected(boolean detected) {
    io.setDetected(detected);
  }
}
