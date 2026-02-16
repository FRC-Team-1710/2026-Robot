package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  public default boolean isConnected() {
    return false;
  }

  public default PhotonPipelineResult getLatestResult() {
    return new PhotonPipelineResult();
  }
}
