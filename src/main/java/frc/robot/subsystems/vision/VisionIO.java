package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  public default void setConsumer(CommandSwerveDrivetrain consumer) {}

  public default void process() {}

  public default boolean isConnected() {
    return false;
  }

  public default PhotonPipelineResult getLatestResult() {
    return new PhotonPipelineResult();
  }

  public default Pose2d getEstimatedGlobalPose() {
    return new Pose2d();
  }
}
