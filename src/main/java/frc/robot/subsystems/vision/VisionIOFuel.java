package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOFuel implements Subsystem {
  @NotLogged private PhotonCamera m_fuelCamera;

  @NotLogged private PhotonPipelineResult m_latestResult;

  @NotLogged private List<PhotonTrackedTarget> m_trackedTargets;

  @NotLogged private PhotonTrackedTarget m_largestTarget;

  public DoubleSupplier yawToLargestTarget = () -> getLargestTarget().getYaw();

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
    m_latestResult = m_fuelCamera.getAllUnreadResults().get(0);

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
    }
  }

  public PhotonTrackedTarget getLargestTarget() {
    return m_largestTarget;
  }
}
