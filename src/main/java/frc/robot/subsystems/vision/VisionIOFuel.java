package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOFuel implements Subsystem {
  @NotLogged private PhotonCamera m_fuelCamera;

  @NotLogged private List<PhotonPipelineResult> m_latestResult;

  @NotLogged private List<PhotonTrackedTarget> m_trackedTargets;

  @NotLogged private PhotonTrackedTarget m_largestTarget;
  @NotLogged private PhotonTrackedTarget m_bestTarget;
  @Logged public double bestCost = 0.0;

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
    m_latestResult = m_fuelCamera.getAllUnreadResults();

    // Gather largest target/cluster of fuel and store all targets
    if (!m_latestResult.isEmpty()) {
      m_trackedTargets = m_latestResult.get(0).getTargets();
      double area = 0;
      double lowestCost = Double.MAX_VALUE;
      for (PhotonTrackedTarget cluster : m_trackedTargets) {
        if (cluster.getArea() > area) {
          area = cluster.getArea();
          m_largestTarget = cluster;
        }

        // Finding the best target based on a cost function
        double currentCost = findCost(cluster);
        if (currentCost < lowestCost) {
          lowestCost = currentCost;
          m_bestTarget = cluster;
          this.bestCost = currentCost;
        }
      }
    }
  }

  private double findCost(PhotonTrackedTarget target) {
    // Cost = w1 * (abs(yaw)/yaw) + w2 * (distance/maxDistance) - w3 * normalizedArea

    return VisionConstants.FUEL_CAMERA_COST_WEIGHT_1
            * Math.abs(target.getYaw())
            / (VisionConstants.FUEL_CAMERA_FOV_H / 2)
        + VisionConstants.FUEL_CAMERA_COST_WEIGHT_2
            * ((target.getPitch() + VisionConstants.FUEL_CAMERA_FOV_V / 2)
                / VisionConstants.FUEL_CAMERA_MAX_DISTANCE)
        - VisionConstants.FUEL_CAMERA_COST_WEIGHT_3
            * target.getArea()
            / Math.pow(Math.cos(Math.toRadians(target.getPitch())), 2);
  }

  public PhotonTrackedTarget getLargestTarget() {
    if (m_largestTarget == null) {
      return new PhotonTrackedTarget();
    }
    return m_largestTarget;
  }

  public PhotonTrackedTarget getBestTarget() {
    if (m_bestTarget == null) {
      return new PhotonTrackedTarget();
    }
    return m_bestTarget;
  }
}
