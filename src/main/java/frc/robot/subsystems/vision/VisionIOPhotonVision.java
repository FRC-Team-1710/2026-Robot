package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera m_camera;
  private final Transform3d m_robotToCamera;

  private final PhotonPoseEstimator photonEstimator =
      new PhotonPoseEstimator(VisionConstants.kTagLayout, m_robotToCamera);

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.m_camera = new PhotonCamera(cameraName);
    this.m_robotToCamera = robotToCamera;
  }

  @Override
  public boolean isConnected() {
    return m_camera.isConnected();
  }

  @Override
  public PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }
}
