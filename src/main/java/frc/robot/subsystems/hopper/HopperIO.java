package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public class HopperIOInputs {
    public double motorTemp = 0.0;
    public double motorVelocity = 0.0;
    public double motorCurrent = 0.0;
    public double motorVolts = 0.0;

    public boolean motorConnected = false;
  }

  public default void setTrackPercent(double percent) {}

  public default void setVelocity(double radPerSec) {}

  public default void stop() {}

  public default void eject(double ejectValue) {}

  public default void updateInputs(HopperIOInputs inputs) {}
}
