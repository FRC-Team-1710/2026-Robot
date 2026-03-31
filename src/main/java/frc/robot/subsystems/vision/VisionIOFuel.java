package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOFuel extends SubsystemBase {
  @NotLogged private final PhotonCamera m_fuelCamera;

  @NotLogged private List<PhotonPipelineResult> m_latestResult;

  @NotLogged private List<PhotonTrackedTarget> m_trackedTargets;

  @NotLogged private PhotonTrackedTarget m_largestTarget;
  @NotLogged private PhotonTrackedTarget m_bestTarget;
  @Logged public double bestCost = 0.0;

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

  public Rotation2d getLatestRotationTarget() {
    return m_rotationTarget;
  }

  public PhotonTrackedTarget getBestTarget() {
    if (m_bestTarget == null) {
      return new PhotonTrackedTarget();
    }
    return m_bestTarget;
  }

  public PhotonTrackedTarget getBestTarget() {
    if (m_bestTarget == null) {
      return new PhotonTrackedTarget();
    }
    return m_bestTarget;
  }
}
