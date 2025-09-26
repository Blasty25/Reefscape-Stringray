package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIOCandle implements LEDIO {
  private final CANdle candle;
  private final CANdleConfiguration config = new CANdleConfiguration();

  private Animation currentAnimation = LEDConstants.disabledAnim;

  public LEDIOCandle() {
    candle = new CANdle(LEDConstants.candleId, "rio");

    config.brightnessScalar = 0.5;
    config.disableWhenLOS = false;
    config.enableOptimizations = true;

    candle.configAllSettings(config);
  }

  @Override
  public void updateInputs(LEDIOInputsAutoLogged inputs) {
    inputs.appliedVolts = candle.get5VRailVoltage();
    inputs.current = candle.getCurrent();
    inputs.temperature = candle.getTemperature();
  }

  @Override
  public void set(Color color, int i) {
    candle.setLEDs(((int) color.red), ((int) color.green), ((int) color.blue), 255, i, 1);
  }

  @Override
  public void set(Color color) {
    candle.setLEDs(
        ((int) color.red * 255),
        ((int) color.green * 255),
        ((int) color.blue * 255),
        255,
        1,
        LEDConstants.length);
  }

  @Override
  public void set(Animation animation) {
    if (currentAnimation != animation) {
      candle.animate(animation);
      currentAnimation = animation;
    }
  }
}
