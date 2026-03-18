package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOFuel extends SubsystemBase {
  @NotLogged private final PhotonCamera m_fuelCamera;

  @NotLogged private PhotonPipelineResult m_latestResult;

  @NotLogged private List<PhotonTrackedTarget> m_trackedTargets;

  @NotLogged private PhotonTrackedTarget m_largestTarget;

  @NotLogged private Rotation2d m_rotationTarget = Rotation2d.kZero;

  @NotLogged private final Supplier<Rotation2d> m_robotRotationSupplier;

  public VisionIOFuel(String cameraName, Supplier<Rotation2d> rotationSupplier) {
    m_fuelCamera = new PhotonCamera(cameraName);
    m_robotRotationSupplier = rotationSupplier;
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

        m_rotationTarget =
            m_robotRotationSupplier.get().plus(Rotation2d.fromDegrees(m_largestTarget.getYaw()));
      }
    }
  }

  public Rotation2d getLatestRotationTarget() {
    return m_rotationTarget;
  }
}
