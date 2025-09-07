package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public class ClimbIOInputs {
    public boolean motorConnected = false;
    public double motorPositionRad = 0.0;
    public double targetPositionRad = 0.0;
    public double motorVelocityRadPerSec = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorStatorCurrentAmps = 0.0;
    public double motorSupplyCurrentAmps = 0.0;
    public double motorTempCelsius = 0.0;
  }

  public default void updateInputs(final ClimbIOInputsAutoLogged inputs) {}

  public default void setPosition(final Rotation2d position) {}

  public default void setVoltage(final double volts) {}

  public default void stop() {}

  public default void resetEncoder() {}

  public default void resetEncoder(final double position) {}

  public default void setBrakeMode(boolean enabled) {}
}
