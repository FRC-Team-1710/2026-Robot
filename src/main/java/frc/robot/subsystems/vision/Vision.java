package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;

/**
 * Vision subsystem responsible for: reading AprilTag detections from PhotonVision; estimating robot
 * field pose; dynamically scaling measurement trust; and feeding vision measurements into the
 * drivetrain pose estimator
 *
 * <p>One instance of this class represents a single physical camera.
 */
public class Vision extends SubsystemBase {

  private final PhotonCamera camera;

  private final PhotonPoseEstimator poseEstimator;

  private final CommandSwerveDrivetrain drivetrain;

  // === Vision state calculated each cycle ===
  // These values are updated from PhotonVision and optionally replayed in simulation.

  private Pose2d robotPose = new Pose2d();

  private double robotPoseTimestamp = 0.0;

  private int tagCount = 0;

  private double avgTagDistance = 0.0;

  private double ambiguity = 0.0;

  private final String m_logPath;

  private final HootAutoReplay autoReplay;

  /**
   * @param cameraName Name of the PhotonVision camera (must match NT name exactly)
   * @param robotToCamera Transform from robot center to camera (meters, radians)
   * @param drivetrain Reference to drivetrain for pose fusion
   */
  public Vision(String cameraName, Transform3d robotToCamera, CommandSwerveDrivetrain drivetrain) {

    m_logPath = cameraName + "/";

    this.drivetrain = drivetrain;

    camera = new PhotonCamera(cameraName);

    poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.kAprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

    autoReplay =
        new HootAutoReplay()
            .withStruct(
                "PhotonVision/" + cameraName + "/RobotPose",
                Pose2d.struct,
                () -> robotPose,
                val -> robotPose = val.value)
            .withDouble(
                "PhotonVision/" + cameraName + "/RobotPoseTimestamp",
                () -> robotPoseTimestamp,
                val -> robotPoseTimestamp = val.value)
            .withInteger(
                "PhotonVision/" + cameraName + "/TagCount",
                () -> tagCount,
                val -> tagCount = val.value.intValue())
            .withDouble(
                "PhotonVision/" + cameraName + "/AvgTagDistance",
                () -> avgTagDistance,
                val -> avgTagDistance = val.value)
            .withDouble(
                "PhotonVision/" + cameraName + "/Ambiguity",
                () -> ambiguity,
                val -> ambiguity = val.value)
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

    autoReplay.update();
    processInputs();
  }

  /**
   * Pulls latest AprilTag detection results and converts them into a field-relative Pose2d
   * estimate.
   *
   * <p>If no valid pose is found, vision state is reset.
   */
  private void fetchInputs() {

    PhotonPipelineResult result = camera.getLatestResult();

    if (!result.hasTargets()) {
      reset();
      return;
    }

    Optional<EstimatedRobotPose> estimate = poseEstimator.update(result);

    if (estimate.isEmpty()) {
      reset();
      return;
    }

    EstimatedRobotPose visionEstimate = estimate.get();

    Robot.telemetry().log(m_logPath + "RawPose", visionEstimate.estimatedPose, Pose3d.struct);

    robotPose = visionEstimate.estimatedPose.toPose2d();
    robotPoseTimestamp = visionEstimate.timestampSeconds;
    tagCount = result.getTargets().size();

    avgTagDistance =
        result.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);

    ambiguity = result.getBestTarget().getPoseAmbiguity();
  }

  private void processInputs() {

    if (tagCount == 0 || robotPoseTimestamp == 0.0) {
      return;
    }
    // Reject single-tag solutions with high ambiguity.
    // Multi-tag solutions are inherently more stable.
    if (tagCount == 1 && ambiguity > VisionConstants.MAX_TAG_AMBIGUITY) {
      return;
    }
    // Dynamically scale measurement
    // More tags = more trust
    // Closer = more trust
    // Larger std dev = less influence in pose estimator.
    double xyStdDev = VisionConstants.BASE_XY_STD_DEV / tagCount;
    double thetaStdDev = VisionConstants.BASE_THETA_STD_DEV / tagCount;
    // Squared distance scaling penalizes far-away tag estimates heavily,
    // since pose error grows nonlinearly with distance.
    double distanceScale = Math.pow(avgTagDistance, 2);

    Robot.telemetry().log(m_logPath + "AcceptedPose", robotPose, Pose2d.struct);

    xyStdDev *= distanceScale;
    thetaStdDev *= distanceScale;
    // Inject measurement into drivetrain pose estimator.
    // Std deviations control how much the estimator trusts vision vs odometry.
    drivetrain.addVisionMeasurement(
        new Pose2d(robotPose.getTranslation(), drivetrain.getRotation()),
        robotPoseTimestamp,
        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }

  /**
   * Clears current vision state when no valid estimate is available. Prevents stale measurements
   * from being reused.
   */
  private void reset() {
    robotPose = new Pose2d();
    robotPoseTimestamp = 0.0;
    tagCount = 0;
    avgTagDistance = 0.0;
    ambiguity = 0.0;
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public double getRobotPoseTimestamp() {
    return robotPoseTimestamp;
  }

  public int getTagCount() {
    return tagCount;
  }

  public double getAvgTagDistance() {
    return avgTagDistance;
  }

  public double getAmbiguity() {
    return ambiguity;
  }
}
