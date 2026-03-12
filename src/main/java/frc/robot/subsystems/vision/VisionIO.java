package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.VisionUtil.VisionMeasurement;

public interface VisionIO {
  public default void fetchInputs() {}

  public default VisionMeasurement processInputs() {
    return new VisionMeasurement(new Pose2d(), 0.0, null);
  }

  public default boolean isConnected() {
    return false;
  }
}
