package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
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
public class Vision implements Subsystem {

  public static record VisionMeasurement(
      String cameraName,
      Pose2d pose,
      double timestampSeconds,
      int tagCount,
      double avgTagDistance,
      double ambiguity,
      double xyStdDev,
      double thetaStdDev) {}

  private final PhotonCamera m_camera;

  private final PhotonPoseEstimator m_poseEstimator;

  // === Vision state calculated each cycle ===
  // These values are updated from PhotonVision and optionally replayed in simulation.

  private Pose2d m_robotPose = new Pose2d();

  private double m_robotPoseTimestamp = 0.0;

  private int m_tagCount = 0;

  private double m_avgTagDistance = 0.0;

  private double m_ambiguity = 0.0;

  private double m_xyStdDev = 0.0;

  private double m_thetaStdDev = 0.0;

  private Optional<VisionMeasurement> m_pendingMeasurement = Optional.empty();

  private final String m_logPath;

  private final HootAutoReplay m_autoReplay;

  private final String cameraName;

  // private final Set<Integer> rejectTagIds =
  //     Set.of(1, 6, 17, 22); // Example tag IDs to reject (e.g., tags on the field perimeter)

  /**
   * @param cameraName Name of the PhotonVision camera (must match NT name exactly)
   * @param robotToCamera Transform from robot center to camera (meters, radians)
   * @param drivetrain Reference to drivetrain for pose fusion
   */
  public Vision(String cameraName, Transform3d robotToCamera) {
    m_logPath = cameraName + "/";

    m_camera = new PhotonCamera(cameraName);

    m_poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.kAprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

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
    SmartDashboard.putBoolean(cameraName + "/rejectWhenDisabled", false);
    this.cameraName = cameraName;
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
  private void fetchInputs() {
    m_pendingMeasurement = Optional.empty();

    PhotonPipelineResult result = m_camera.getLatestResult();
    // Filtering based on the rejected tags
    // PhotonPipelineResult filteredResult =
    //     new PhotonPipelineResult(
    //         result.metadata,
    //         result.getTargets().stream()
    //             .filter(t -> !rejectTagIds.contains(t.getFiducialId()))
    //             .toList(),
    //         result.getMultiTagResult().isPresent() ? result.getMultiTagResult() :
    // Optional.empty());
    if (!result.hasTargets()) {
      reset();
      return;
    }

    if (SmartDashboard.getBoolean(cameraName + "/rejectWhenDisabled", false)
        && DriverStation.isDisabled()) {
      reset();
      return;
    }

    Optional<EstimatedRobotPose> estimate = m_poseEstimator.update(result);

    if (estimate.isEmpty()) {
      reset();
      return;
    }

    EstimatedRobotPose visionEstimate = estimate.get();
    Pose2d visionPose = visionEstimate.estimatedPose.toPose2d();

    if (isFrameRejected(visionPose, visionEstimate.timestampSeconds)) {
      reset();
      return;
    }

    Robot.telemetry().log(m_logPath + "RawPose", visionEstimate.estimatedPose, Pose3d.struct);

    m_robotPose = visionPose;
    m_robotPoseTimestamp = visionEstimate.timestampSeconds;
    m_tagCount = result.getTargets().size();

    m_avgTagDistance =
        result.getTargets().stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);

    m_ambiguity = result.getBestTarget().getPoseAmbiguity();
  }

  private void processInputs() {
    m_pendingMeasurement = Optional.empty();

    if (m_tagCount == 0 || m_robotPoseTimestamp == 0.0 || m_robotPose == Pose2d.kZero) {
      return;
    }
    // Reject single-tag solutions with high ambiguity.
    // Multi-tag solutions are inherently more stable.
    if (m_tagCount == 1 && m_ambiguity > VisionConstants.MAX_TAG_AMBIGUITY) {
      return;
    }
    // Dynamically scale measurement
    // More tags = more trust
    // Closer = more trust
    // Larger std dev = less influence in pose estimator.
    double xyStdDev = VisionConstants.BASE_XY_STD_DEV / m_tagCount;

    if (m_tagCount == 1 && !DriverStation.isAutonomous()) {
      xyStdDev *= 1.5; // Single tag is less reliable, so start with higher std dev
    }
    // Squared distance scaling penalizes far-away tag estimates heavily,
    // since pose error grows nonlinearly with distance.
    double distanceScale = Math.pow(m_avgTagDistance, 2);

    xyStdDev *= distanceScale;

    m_xyStdDev =
        Math.max(
            VisionConstants.MIN_XY_STD_DEV, Math.min(xyStdDev, VisionConstants.MAX_XY_STD_DEV));
    m_thetaStdDev = VisionConstants.BASE_THETA_STD_DEV / Math.max(1, m_tagCount);
    m_thetaStdDev =
        Math.max(
            VisionConstants.MIN_THETA_STD_DEV,
            Math.min(m_thetaStdDev, VisionConstants.MAX_THETA_STD_DEV));

    Robot.telemetry().log(m_logPath + "XYStdDev", m_xyStdDev);
    Robot.telemetry().log(m_logPath + "ThetaStdDev", m_thetaStdDev);

    Robot.telemetry().log(m_logPath + "AcceptedPose", m_robotPose, Pose2d.struct);
    m_pendingMeasurement =
        Optional.of(
            new VisionMeasurement(
                cameraName,
                m_robotPose,
                m_robotPoseTimestamp,
                m_tagCount,
                m_avgTagDistance,
                m_ambiguity,
                m_xyStdDev,
                m_thetaStdDev));
  }

  private boolean isFrameRejected(Pose2d visionPose, double timestampSeconds) {
    double ageSeconds = Utils.getCurrentTimeSeconds() - timestampSeconds;
    if (ageSeconds > VisionConstants.MAX_FRAME_AGE_SECONDS) {
      Robot.telemetry().log(m_logPath + "FrameRejected/AgeSeconds", ageSeconds);
      return true;
    }

    double fieldLength = FieldConstants.kFieldLength.in(edu.wpi.first.units.Units.Meters);
    double fieldWidth = FieldConstants.kFieldWidth.in(edu.wpi.first.units.Units.Meters);
    double x = visionPose.getX();
    double y = visionPose.getY();
    boolean outOfBounds =
        x < -VisionConstants.FIELD_MARGIN_METERS
            || x > fieldLength + VisionConstants.FIELD_MARGIN_METERS
            || y < -VisionConstants.FIELD_MARGIN_METERS
            || y > fieldWidth + VisionConstants.FIELD_MARGIN_METERS;
    if (outOfBounds) {
      Robot.telemetry().log(m_logPath + "FrameRejected/OutOfBounds", visionPose, Pose2d.struct);
    }

    return outOfBounds;
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
    m_xyStdDev = 0.0;
    m_thetaStdDev = 0.0;
    m_pendingMeasurement = Optional.empty();
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

  /** Returns the current X/Y standard deviation chosen for the latest accepted vision pose. */
  public double getXyStdDev() {
    return m_xyStdDev;
  }

  /** Returns the current theta standard deviation chosen for the latest accepted vision pose. */
  public double getThetaStdDev() {
    return m_thetaStdDev;
  }

  /** Returns the latest accepted vision measurement for this camera, if any. */
  public Optional<VisionMeasurement> getPendingMeasurement() {
    return m_pendingMeasurement;
  }

  public boolean isConnected() {
    return m_camera.isConnected();
  }
}
