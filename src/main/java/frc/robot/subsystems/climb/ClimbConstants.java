package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ClimbConstants {
  public static final int talon = 50;

  public static final boolean inverted = false;

  public static final double mass = Units.lbsToKilograms(25.1);

  public static final Rotation2d stow = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d ready = Rotation2d.fromDegrees(33);
  public static final Rotation2d climbed = Rotation2d.fromDegrees(94);

  public static final double kPTalon = 150;

  public static final double fast = 2.0;
  public static final double slow = 0.5;

  public static final double gearing = (9.0 / 1.0) * (5.0 / 1.0) * (58.0 / 18.0) * (28.0 / 12.0);
  public static final double moi = 0;
  public static final double length = Units.inchesToMeters(8.83);
  public static final double tolerance = .01;

  public static final int sparkCurrent = 80;
  public static final int talonStatorCurrent = 80;

  public static final double maxVoltage = 12;

  public static final double visualizerOffset = 0.0;
}
