package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOFuel extends SubsystemBase {
  @NotLogged private PhotonCamera m_fuelCamera;

  @NotLogged private PhotonPipelineResult m_latestResult;

  @NotLogged private List<PhotonTrackedTarget> m_trackedTargets;

  @NotLogged private PhotonTrackedTarget m_largestTarget;

  // public DoubleSupplier yawToLargestTarget = () -> getLargestTarget().getYaw();

  public VisionIOFuel(String cameraName) {
    m_fuelCamera = new PhotonCamera(cameraName);
  }

  @Override
  public void periodic() {
    getLatestResult();
  }

  /**
   * Gathers the latest result from the fuel camera.
   *
   * <p>ONLY CALL THIS ONCE PER LOOP
   */
  private void getLatestResult() {
    // Get the latest result (This is what can only be called once)
    var m_latestResults = m_fuelCamera.getAllUnreadResults();
    if (m_latestResults.size() > 0) {
      m_latestResult = m_latestResults.get(0);
      // Gather largest target/cluster of fuel and store all targets
      if (m_latestResult.hasTargets()) {
        m_trackedTargets = m_latestResult.getTargets();
        double area = 0;
        for (PhotonTrackedTarget cluster : m_trackedTargets) {
          if (cluster.getArea() > area) {
            area = cluster.getArea();
            m_largestTarget = cluster;
          }
        }
      } else {
        m_largestTarget = null;
      }
    } else {
      m_largestTarget = null;
    }
  }

  // public PhotonTrackedTarget getLargestTarget() {
  //   if (m_largestTarget == null) {
  //     return new PhotonTrackedTarget();
  //   }
  //   return m_largestTarget;
  // }

  public double getLargestTargetYaw() {
    return m_largestTarget == null ? 0 : m_largestTarget.getYaw();
  }
}
