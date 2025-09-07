package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotState;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  RobotState state = RobotState.Idle;

  private Color color;
  private Animation disabledAnimation;

  private Animation animation;

  public LED(LEDIO io) {
    this.io = io;

    color =
        DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red ? Color.kDarkRed : Color.kNavy)
            .orElse(Color.kDarkRed);
    disabledAnimation =
        new SingleFadeAnimation(
            (int) (color.red * 255),
            (int) (color.green * 255),
            (int) (color.blue * 255),
            0,
            0,
            LEDConstants.length);
    io.set(disabledAnimation);
    animation = disabledAnimation;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  private void setIndex(int i, Color color) {
    io.set(color, i);
  }

  private Color solid(Color color) {
    io.set(color);
    return color;
  }

  public Color strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    if ((c1On && c1 == null) || (!c1On && c2 == null)) return null;
    return solid(c1On ? c1 : c2);
  }

  public Color breath(Color c1, Color c2, double duration, double timestamp) {
    Color color = breathCalculate(c1, c2, duration, timestamp);
    solid(color);
    return color;
  }

  private Color breathCalculate(Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    Color color = new Color(red, green, blue);
    return color;
  }

  public void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = LEDConstants.length - 1; i >= LEDConstants.length; i--) {
      x += xDiffPerLed;
      x %= 180.0;
      setIndex(i, Color.fromHSV((int) x, 255, 255));
    }
  }

  public void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = LEDConstants.length - 1; i >= 0; i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setIndex(i, new Color(red, green, blue));
    }
  }

  public Command set(Animation animation) {
    return this.run(
        () -> {
          io.set(animation);
        });
  }

  public Command setState(RobotState state) {
    return this.run(
        () -> {
          switch (state) {
            case Idle:
              wave(Color.kBlue, Color.kYellow, 20, 3);
              // LEDConstants.flowAnimation(io, this);
            case Manual_Score:
              animation = LEDConstants.manualScoreAnimation;
              break;
            case SetElevatorSetpoint:
              animation = LEDConstants.setElevatorAnimation;
              break;
            case Shoot:
              animation = LEDConstants.shootCoralAnimation;
              break;
            case Eject:
              animation = LEDConstants.ejectAnimation;
              break;
            case Pre_Algae:
              animation = LEDConstants.algaeReadyAnimation;
              break;
            case Climb_Pull:
              animation = LEDConstants.climbReadyAnimation;
              break;
            case Climb_Ready:
              if (DriverStation.isDisabled()) {
                animation = LEDConstants.climbedAnimation;
              } else {
                animation = LEDConstants.endAnimation;
              }
              break;
            case Algae_Armed:
            case Algae_Intake:
            default:
              LEDConstants.flowAnimation(io, this);
              break;
          }
          io.set(animation);
        });
  }
}
