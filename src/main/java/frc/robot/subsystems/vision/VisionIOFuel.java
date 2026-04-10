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
      double lowestCost = Double.MAX_VALUE;
      for (PhotonTrackedTarget cluster : m_trackedTargets) {
        double cost = findCost(cluster);
        if (cost < lowestCost) {
          lowestCost = cost;
          m_bestTarget = cluster;
        }
      }
    }
  }

  /**
   * Calculates the cost of a cluster using a specific area
   *
   * @param target A cluster of fuel that is returned from the fuel camera of type {@link
   *     PhotonTrackedTarget}
   * @return {@code double} cost value (lower is a more favorable cluster)
   */
  private double findCost(PhotonTrackedTarget target) {
    // Cost = w1 * (abs(yaw)/(horizontalFOV/2)) + w2 * ((pitch + verticalFOV/2)/maxDistance)
    // - w3 * normalizedArea

    return VisionConstants.FUEL_CAMERA_COST_WEIGHT_1
            * Math.abs(target.getYaw())
            / (VisionConstants.FUEL_CAMERA_FOV_H / 2)
        + VisionConstants.FUEL_CAMERA_COST_WEIGHT_2
            * ((target.getPitch() + VisionConstants.FUEL_CAMERA_FOV_V / 2)
                / VisionConstants.FUEL_CAMERA_MAX_DISTANCE)
        - VisionConstants.FUEL_CAMERA_COST_WEIGHT_3
            * target.getArea()
            / Math.pow(
                Math.cos(Math.toRadians(target.getPitch() + VisionConstants.FUEL_CAMERA_FOV_V / 2)),
                2);
  }

  public Rotation2d getLatestRotationTarget() {
    return m_robotRotationSupplier.get().plus(Rotation2d.fromDegrees(m_bestTarget.getYaw()));
  }

  public PhotonTrackedTarget getBestTarget() {
    if (m_bestTarget == null) {
      return new PhotonTrackedTarget();
    }
    return m_bestTarget;
  }
}
