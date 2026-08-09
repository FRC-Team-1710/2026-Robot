package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SubsystemConstants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;

/** One instance of this class represents a single physical camera. */
@SuppressWarnings("removal")
public class Vision {

  private final PhotonCamera m_camera;

  private final PhotonPoseEstimator m_poseEstimator;

  private final CommandSwerveDrivetrain m_drivetrain;

  private Pose3d m_rawPose = new Pose3d();

  private Pose2d m_robotPose = new Pose2d();

  private double m_robotPoseTimestamp = 0.0;

  private int m_tagCount = 0;

  private double m_ambiguity = 0.0;

  private double m_avgTagDistance = 0.0;

  private boolean m_accepted = false;

  private double m_xyStdDev = 0.0;

  private double m_thetaStdDev = 100000.0;

  private boolean m_hasEnabled = false;

  /**
   * @param cameraName Name of the PhotonVision camera (must match NT name exactly)
   * @param robotToCamera Transform from robot center to camera (meters, radians)
   * @param drivetrain Reference to drivetrain for pose fusion
   */
  public Vision(String cameraName, Transform3d robotToCamera, CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;

    m_camera = new PhotonCamera(cameraName);

    m_poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.kAprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * Runs every scheduler loop. 1. Fetch fresh vision data (unless replaying logs) 2. Process and
   * inject pose measurements into drivetrain
   */
  public void periodic() {
    reset();
    fetchInputs();
    processInputs();
    log();
  }

  /**
   * Pulls latest AprilTag detection results and converts them into a field-relative Pose2d
   * estimate.
   */
  private void fetchInputs() {

    PhotonPipelineResult result = m_camera.getLatestResult();

    if (!result.hasTargets()) {
      return;
    }

    Optional<EstimatedRobotPose> estimate = m_poseEstimator.update(result);

    if (estimate.isEmpty()) {
      return;
    }

    EstimatedRobotPose visionEstimate = estimate.get();

    // Flip by 180 because drivetrain was id'd incorrect and not enough time to fix before comp.
    m_rawPose =
        new Pose3d(
            visionEstimate.estimatedPose.getTranslation(),
            visionEstimate.estimatedPose.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
    m_robotPose = m_rawPose.toPose2d();
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

    if (m_tagCount == 0 || m_robotPoseTimestamp == 0.0 || m_robotPose == Pose2d.kZero) {
      return;
    }

    // Reject single-tag solutions with high ambiguity.
    // Multi-tag solutions are inherently more stable.
    if (m_tagCount == 1
        && (m_ambiguity > VisionConstants.Software.MAX_TAG_AMBIGUITY
            || DriverStation.isAutonomous())) {
      return;
    }

    // Dynamically scale measurement
    // More tags = more trust
    // Closer = more trust
    // Larger std dev = less influence in pose estimator.
    m_xyStdDev = VisionConstants.Software.BASE_XY_STD_DEV / Math.pow(m_tagCount - 1, 2);

    if (m_tagCount == 1) {
      // Single tag is less reliable, so start with higher std dev
      m_xyStdDev *= 1.5;

      // Squared distance scaling penalizes far-away tag estimates heavily,
      // since pose error grows non-linearly with distance.
      m_xyStdDev *= Math.pow(m_avgTagDistance, 2);
    } else if (m_tagCount == 2) {
      // ^1.5 distance scaling penalizes far-away tag estimates more heavily,
      // since pose error grows non-linearly with distance.
      m_xyStdDev *= Math.pow(m_avgTagDistance, 1.5);
    } else {
      // only trust rotation if we have many tags
      // when 3 tags are seen, not all tags are on the same plane so the estimate becomes more
      // significantly reliable
      m_thetaStdDev = VisionConstants.Software.BASE_THETA_STD_DEV / ((m_tagCount - 2) / 2);
    }

    if (DriverStation.isEnabled() || m_hasEnabled) {
      // Inject measurement into drivetrain pose estimator.
      // Std deviations control how much the estimator trusts vision vs odometry.
      m_drivetrain.addVisionMeasurement(
          m_robotPose,
          m_robotPoseTimestamp,
          VecBuilder.fill(m_xyStdDev, m_xyStdDev, m_thetaStdDev));
      m_accepted = true;
      m_hasEnabled = true;
      return;
    }
  }

  /**
   * Clears current vision state when no valid estimate is available. Prevents stale measurements
   * from being reused.
   */
  private void reset() {
    m_rawPose = new Pose3d();
    m_robotPose = new Pose2d();
    m_robotPoseTimestamp = 0.0;
    m_tagCount = 0;
    m_ambiguity = 0.0;
    m_avgTagDistance = 0.0;
    m_accepted = false;
    m_xyStdDev = 0.0;
    m_thetaStdDev = 100000.0;
  }

  private void log() {
    Robot.telemetry().log("Vision/" + m_camera.getName() + "/RawPose", m_rawPose, Pose3d.struct);
    Robot.telemetry()
        .log("Vision/" + m_camera.getName() + "/RobotPose", m_robotPose, Pose2d.struct);
    Robot.telemetry()
        .log("Vision/" + m_camera.getName() + "/RobotPoseTimestamp", m_robotPoseTimestamp);
    Robot.telemetry().log("Vision/" + m_camera.getName() + "/TagCount", m_tagCount);
    Robot.telemetry().log("Vision/" + m_camera.getName() + "/Ambiguity", m_ambiguity);
    Robot.telemetry().log("Vision/" + m_camera.getName() + "/AvgTagDistance", m_avgTagDistance);
    Robot.telemetry().log("Vision/" + m_camera.getName() + "/Accepted", m_accepted);
    Robot.telemetry().log("Vision/" + m_camera.getName() + "/XYStdDev", m_xyStdDev);
    Robot.telemetry().log("Vision/" + m_camera.getName() + "/ThetaStdDev", m_thetaStdDev);
  }
}
