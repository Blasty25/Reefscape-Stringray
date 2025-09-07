package frc.robot.util;

/*This class will list all ids of every motor on this robot */
public class RobotMap {

  public static class ElevatorMap {
    public static final int leftID = 11;
    public static final int rightID = 10;
  }

  public static class OuttakeMap {
    public static final int outtakeID = 20;
    public static final int canandcolorID = 21;
  }

  public static class HopperMap {
    public static final int hopperTalon = 30;
    public static final int laser = 31; // Laser can on the hopper, not implement in my code!w
  }

  public static class GripperMap {
    public static final int talon = 40;
    public static final int laser = 41;
  }

  public static class ClimbMap {
    public static final int talon = 50;
  }
}
