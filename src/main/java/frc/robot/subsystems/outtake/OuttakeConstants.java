package frc.robot.subsystems.outtake;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import com.reduxrobotics.sensors.canandcolor.ProximityPeriod;

public class OuttakeConstants {

  public static final int current = 60;
  public static final boolean inverted = false;

  public static final int canandcolor = 21;
  public static final double proximityThreshold = 0.15;

  public static final int laserEffector = 22;
  public static final RangingMode rangingMode = RangingMode.SHORT;
  public static final RegionOfInterest regionOfInterest = new RegionOfInterest(4, 4, 4, 4);

  public static final int laserHandoff = 23;
  public static final int laserBaseLeft = 24;
  public static final int laserBaseRight = 25;

  public static final double index = 3;
  public static final double L1 = 8;
  public static final double L23 = 9;
  public static final double L4 = 3;

  public static final double gearing = 20.0 / 12.0;
  public static final double moi = 0.004;

  public static ProximityPeriod period = ProximityPeriod.k5ms;
}
