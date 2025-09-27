package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotState;
import java.util.Map;

public class LEDConstants {
  public static final int rioId = 0;
  public static final int candleId = 60;
  private static final Alliance kAlliance =
      DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
  public static final double brightness = 0.5;
  public static final int length = 308;

  public static final Map<RobotState, Animation> animMap =
      Map.of(
          RobotState.Idle,
          new SingleFadeAnimation(
              convertColorToInt(Color.kGold)[0],
              convertColorToInt(Color.kGold)[1],
              convertColorToInt(Color.kGold)[2],
              255,
              0.5,
              length),
          RobotState.Intake,
          new StrobeAnimation(
              convertColorToInt(Color.kWhite)[0],
              convertColorToInt(Color.kWhite)[1],
              convertColorToInt(Color.kWhite)[2],
              255,
              0.5,
              length),
          RobotState.SetElevatorSetpoint,
          new SingleFadeAnimation(255, 255, 255, 255, 0.5, length),
          RobotState.Shoot,
          new StrobeAnimation(
              convertColorToInt(Color.kLimeGreen)[0],
              convertColorToInt(Color.kLimeGreen)[1],
              convertColorToInt(Color.kLimeGreen)[2],
              255,
              0.5,
              length),
          RobotState.AlgaeIntake,
          new StrobeAnimation(
              convertColorToInt(Color.kTeal)[0],
              convertColorToInt(Color.kTeal)[1],
              convertColorToInt(Color.kTeal)[2],
              255,
              0.5,
              length),
          RobotState.AlgaeArmed,
          new SingleFadeAnimation(
              convertColorToInt(Color.kTeal)[0],
              convertColorToInt(Color.kTeal)[1],
              convertColorToInt(Color.kTeal)[2],
              255,
              0.5,
              length),
          RobotState.AlgaeSetpoint,
          new StrobeAnimation(
              convertColorToInt(Color.kPurple)[0],
              convertColorToInt(Color.kPurple)[1],
              convertColorToInt(Color.kPurple)[2],
              255,
              0.5,
              length),
          RobotState.Manual_Score,
          new StrobeAnimation(
              convertColorToInt(Color.kBlue)[0],
              convertColorToInt(Color.kBlue)[1],
              convertColorToInt(Color.kBlue)[2],
              255,
              0.5,
              length),
          RobotState.ClimbReady,
          new StrobeAnimation(
              convertColorToInt(Color.kTeal)[0],
              convertColorToInt(Color.kTeal)[1],
              convertColorToInt(Color.kTeal)[2],
              255,
              0.5,
              length),
          RobotState.ClimbPull,
          new StrobeAnimation(
              convertColorToInt(Color.kTeal)[0],
              convertColorToInt(Color.kTeal)[1],
              convertColorToInt(Color.kTeal)[2],
              255,
              0.5,
              length));

  public static final Animation disabledAnim =
      new SingleFadeAnimation(
          (kAlliance == Alliance.Red)
              ? (int) (Color.kRed.red * 255)
              : (int) (Color.kBlue.red * 255),
          (kAlliance == Alliance.Red)
              ? (int) (Color.kRed.green * 255)
              : (int) (Color.kBlue.green * 255),
          (kAlliance == Alliance.Red)
              ? (int) (Color.kRed.blue * 255)
              : (int) (Color.kBlue.blue * 255),
          255,
          0.5,
          length);

  public static int[] convertColorToInt(Color color) {
    return new int[] {(int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)};
  }

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
