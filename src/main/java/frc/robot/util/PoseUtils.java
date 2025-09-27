package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PoseUtils {

  /**
   * @param from starting pose
   * @param to ending pose
   * @return double[] {xOffsetInInches, yOffsetInInches}
   */
  public static double[] getOffsetInches(Pose2d from, Pose2d to) {
    double dxMeters = to.getX() - from.getX();
    double dyMeters = to.getY() - from.getY();
    System.out.println("Drive pose: " + from);
    System.out.println("Dx: " + dxMeters + "Meters" + "Dy: " + dyMeters + "Meters");
    return new double[] {Units.metersToInches(dxMeters), Units.metersToInches(dyMeters)};
  }

  public static Command getOffsets(Supplier<Pose2d> aprilTagPose, Supplier<Pose2d> drivePose) {
    return Commands.runOnce(
        () -> {
          Logger.recordOutput("PoseOffsets/AprilTag", aprilTagPose.get());
          double[] offsets = PoseUtils.getOffsetInches(drivePose.get(), aprilTagPose.get());
          System.out.printf("X offset: %.2f in, Y offset: %.2f in%n", offsets[0], offsets[1]);
        });
  }
}
