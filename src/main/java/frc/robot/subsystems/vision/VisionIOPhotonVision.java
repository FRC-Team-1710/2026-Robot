package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.PoseCameraConfig;
import frc.robot.utils.VisionUtil.VisionMeasurement;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera m_camera;

  private final PhotonPoseEstimator m_poseEstimator;

  // === Vision state calculated each cycle ===
  // These values are updated from PhotonVision and optionally replayed in simulation.

  private Pose2d m_robotPose = new Pose2d();

  private double m_robotPoseTimestamp = 0.0;

  private int m_tagCount = 0;

  private double m_avgTagDistance = 0.0;

  private double m_ambiguity = 0.0;

  private final String m_logPath;

  private final HootAutoReplay m_autoReplay;

  private final Set<Integer> rejectTagIds =
      Set.of(1, 6, 17, 22); // Example tag IDs to reject (e.g., tags on the field perimeter)

  /**
   * @param cameraName Name of the PhotonVision camera (must match NT name exactly)
   * @param robotToCamera Transform from robot center to camera (meters, radians)
   */
  public VisionIOPhotonVision(PoseCameraConfig config) {
    String cameraName = config.name();
    m_logPath = cameraName + "/";

    m_camera = new PhotonCamera(cameraName);

    m_poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.kAprilTags,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            config.robotToCamera());
    m_autoReplay =
        new HootAutoReplay()
            .withStruct(
                "PhotonVision/" + cameraName + "/RobotPose",
                Pose2d.struct,
                () -> m_robotPose,
                val -> m_robotPose = val.value)
            .withDouble(
                "PhotonVision/" + cameraName + "/RobotPoseTimestamp",
                () -> m_robotPoseTimestamp,
                val -> m_robotPoseTimestamp = val.value)
            .withInteger(
                "PhotonVision/" + cameraName + "/TagCount",
                () -> m_tagCount,
                val -> m_tagCount = val.value.intValue())
            .withDouble(
                "PhotonVision/" + cameraName + "/AvgTagDistance",
                () -> m_avgTagDistance,
                val -> m_avgTagDistance = val.value)
            .withDouble(
                "PhotonVision/" + cameraName + "/Ambiguity",
                () -> m_ambiguity,
                val -> m_ambiguity = val.value)
            .withTimestampReplay();
  }

  /**
   * Runs every scheduler loop. 1. Fetch fresh vision data (unless replaying logs) 2. Update
   * replay/log values 3. Process and inject pose measurements into drivetrain
   */
  public void periodic() {
    if (!Utils.isReplay()) {
      fetchInputs();
    }

    m_autoReplay.update();
    processInputs();
  }

  /**
   * Pulls latest AprilTag detection results and converts them into a field-relative Pose2d
   * estimate.
   *
   * <p>If no valid pose is found, vision state is reset.
   */
  public void fetchInputs() {

    PhotonPipelineResult result = m_camera.getLatestResult();
    // Filtering based on the rejected tags
    PhotonPipelineResult filteredResult =
        new PhotonPipelineResult(
            result.metadata,
            result.getTargets().stream()
                .filter(t -> !rejectTagIds.contains(t.getFiducialId()))
                .toList(),
            result.getMultiTagResult().isPresent() ? result.getMultiTagResult() : Optional.empty());
    if (!filteredResult.hasTargets()) {
      reset();
      return;
    }

    Optional<EstimatedRobotPose> estimate = m_poseEstimator.update(filteredResult);

    if (estimate.isEmpty()) {
      reset();
      return;
    }

    EstimatedRobotPose visionEstimate = estimate.get();

    Robot.telemetry().log(m_logPath + "RawPose", visionEstimate.estimatedPose, Pose3d.struct);

    m_robotPose = visionEstimate.estimatedPose.toPose2d();
    m_robotPoseTimestamp = visionEstimate.timestampSeconds;
    m_tagCount = filteredResult.getTargets().size();

    m_avgTagDistance =
        filteredResult.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);

    m_ambiguity = filteredResult.getBestTarget().getPoseAmbiguity();
  }

  public VisionMeasurement processInputs() {

    if (m_tagCount == 0 || m_robotPoseTimestamp == 0.0) {
      return new VisionMeasurement(
          new Pose2d(), m_robotPoseTimestamp, VecBuilder.fill(0.0, 0.0, 0.0));
    }
    // Reject single-tag solutions with high ambiguity.
    // Multi-tag solutions are inherently more stable.
    if (m_tagCount == 1 && m_ambiguity > VisionConstants.MAX_TAG_AMBIGUITY) {
      return new VisionMeasurement(
          new Pose2d(), m_robotPoseTimestamp, VecBuilder.fill(0.0, 0.0, 0.0));
    }
    // Dynamically scale measurement
    // More tags = more trust
    // Closer = more trust
    // Larger std dev = less influence in pose estimator.
    double xyStdDev = VisionConstants.BASE_XY_STD_DEV / m_tagCount;
    double thetaStdDev = VisionConstants.BASE_THETA_STD_DEV / m_tagCount;
    // Squared distance scaling penalizes far-away tag estimates heavily,
    // since pose error grows nonlinearly with distance.
    double distanceScale = Math.pow(m_avgTagDistance, 2);

    Robot.telemetry().log(m_logPath + "AcceptedPose", m_robotPose, Pose2d.struct);

    xyStdDev *= distanceScale;
    thetaStdDev *= distanceScale;
    // Inject measurement into drivetrain pose estimator.
    // Std deviations control how much the estimator trusts vision vs odometry.
    return new VisionMeasurement(
        m_robotPose, m_robotPoseTimestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }

  /**
   * Clears current vision state when no valid estimate is available. Prevents stale measurements
   * from being reused.
   */
  private void reset() {
    m_robotPose = new Pose2d();
    m_robotPoseTimestamp = 0.0;
    m_tagCount = 0;
    m_avgTagDistance = 0.0;
    m_ambiguity = 0.0;
  }

  /** Returns the estimated robot pose. */
  public Pose2d getRobotPose() {
    return m_robotPose;
  }

  /** Returns the timestamp of the robot pose estimate. */
  public double getRobotPoseTimestamp() {
    return m_robotPoseTimestamp;
  }

  /** Returns the number of detected AprilTags. */
  public int getTagCount() {
    return m_tagCount;
  }

  /** Returns the average distance to detected tags. */
  public double getAvgTagDistance() {
    return m_avgTagDistance;
  }

  /** Returns the pose ambiguity of the best target. */
  public double getAmbiguity() {
    return m_ambiguity;
  }

  /** Returns whether the vision camera is connected. */
  public boolean isConnected() {
    return m_camera.isConnected();
  }
}
