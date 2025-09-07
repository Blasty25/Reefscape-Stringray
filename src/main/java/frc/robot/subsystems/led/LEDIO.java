package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public double appliedVolts = 0.0;
    public double current = 0.0;
    public double temperature = 0.0;
  }

  public default void updateInputs(LEDIOInputsAutoLogged inputs) {}

  public default void set(Color color) {}

  public default void set(Color color, int i) {}

  public default void set(Animation animation) {}
}
