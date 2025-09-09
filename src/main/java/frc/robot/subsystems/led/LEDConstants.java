package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LEDConstants {
  public static final int rioId = 0;
  public static final int candleId = 60;

  public static final double brightness = 0.5;
  public static final int length = 308;

  public static final Animation setElevatorAnimation =
      new StrobeAnimation(150, 0, 0, 0, 0.7, length);
  public static final Animation ejectAnimation = new StrobeAnimation(0, 200, 10, 0, 0.75, length);
  public static final Animation climbReadyAnimation =
      new StrobeAnimation(
          (int) Color.kYellow.red,
          (int) Color.kYellow.green,
          (int) Color.kYellow.blue,
          0,
          0.5,
          length);
  public static final Animation outtakeAnimation =
      new StrobeAnimation(
          (int) Color.kGreen.red,
          (int) Color.kGreen.green,
          (int) Color.kGreen.blue,
          0,
          0.5,
          length);
  public static final Animation revFunnelAnimation =
      new StrobeAnimation(
          (int) Color.kOrange.red,
          (int) Color.kOrange.green,
          (int) Color.kOrange.blue,
          0,
          0.5,
          length);

  public static final Animation endAnimation = new RainbowAnimation(1, 0.5, LEDConstants.length);
  public static final Animation manualScoreAnimation =
      new StrobeAnimation(255, 165, 0, 0, 0.9, length);
  public static final Animation shootCoralAnimation =
      new RainbowAnimation(1, 0.8, LEDConstants.length);
  public static final Animation algaeReadyAnimation =
      new SingleFadeAnimation(0, 255, 255, 0, 0, LEDConstants.length);
  public static final Animation climbedAnimation =
      new SingleFadeAnimation(255, 255, 0, 0, 0, LEDConstants.length);

  public static final Animation whiteStrobe =
      new StrobeAnimation(255, 255, 255, 255, 0.8, LEDConstants.length);

  public static final double waveExponent = 0.4;

  public static Command flowAnimation(LEDIO io, LED led) {
    ColorFlowAnimation blue =
        new ColorFlowAnimation(0, 0, 255, 0, 0.8, LEDConstants.length, Direction.Forward);
    ColorFlowAnimation yellow =
        new ColorFlowAnimation(255, 255, 0, 0, 0.8, LEDConstants.length, Direction.Backward);
    return Commands.run(
        () ->
            Commands.sequence(
                Commands.runOnce(() -> io.set(blue)),
                Commands.waitSeconds(2.0),
                Commands.runOnce(() -> io.set(yellow))),
        led);
  }
}
