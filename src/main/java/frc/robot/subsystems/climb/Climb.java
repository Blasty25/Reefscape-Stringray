package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final Alert climbDisconnected =
      new Alert("Climb motor disconnected!", Alert.AlertType.kWarning);

  private BooleanSupplier coastOverride = () -> false;

  @AutoLogOutput(key = "Climb/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  public Climb(ClimbIO io) {
    this.io = io;
    io.resetEncoder();
    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    climbDisconnected.set(!inputs.motorConnected);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0);
    }

    boolean coast = coastOverride.getAsBoolean() && DriverStation.isDisabled();
    setBrakeMode(!coast);
    Logger.recordOutput("Climb/CoastOverride", !coast);
  }

  public Command setPosition(Rotation2d position) {
    return this.run(
        () -> {
          inputs.targetPositionRad = position.getRadians();
          io.setPosition(position);
        });
  }

  public Command resetEncoder() {
    return Commands.runOnce(() -> io.resetEncoder()).ignoringDisable(true);
  }

  public Command setBrakeMode(boolean enabled) {
    return Commands.runOnce(() -> io.setBrakeMode(brakeModeEnabled));
  }

  public void setCoastOverride(BooleanSupplier coastOverride) {
    this.coastOverride = coastOverride;
  }

  public double getPositionRad() {
    return inputs.motorPositionRad;
  }

  public double getTargetPositionRad() {
    return inputs.targetPositionRad;
  }
}
