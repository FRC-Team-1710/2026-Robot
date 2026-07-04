package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
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
public class Vision implements Subsystem {

  @AutoLog
  public static class VisionInputs {
    public Pose2d robotPose = new Pose2d();
    public double robotPoseTimestamp = 0.0;
    public int tagCount = 0;
    public double avgTagDistance = 0.0;
    public double ambiguity = 0.0;
  }

  private final PhotonCamera m_camera;

  private final PhotonPoseEstimator m_poseEstimator;

  private final CommandSwerveDrivetrain m_drivetrain;

  // === Vision state calculated each cycle ===
  private Pose2d m_robotPose = new Pose2d();
  private double m_robotPoseTimestamp = 0.0;
  private int m_tagCount = 0;
  private double m_avgTagDistance = 0.0;
  private double m_ambiguity = 0.0;

  private final String m_logPath;

  private final String cameraName;

  /**
   * @param cameraName Name of the PhotonVision camera (must match NT name exactly)
   * @param robotToCamera Transform from robot center to camera (meters, radians)
   * @param drivetrain Reference to drivetrain for pose fusion
   */
  public Vision(String cameraName, Transform3d robotToCamera, CommandSwerveDrivetrain drivetrain) {
    m_logPath = cameraName + "/";

    this.m_drivetrain = drivetrain;

    m_camera = new PhotonCamera(cameraName);

    m_poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.kAprilTags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

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

    processInputs();
  }

  /**
   * Pulls latest AprilTag detection results and converts them into a field-relative Pose2d
   * estimate.
   */
  private void fetchInputs() {

    PhotonPipelineResult result = m_camera.getLatestResult();
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

    Logger.recordOutput(m_logPath + "RawPose", visionEstimate.estimatedPose, Pose3d.struct);

    m_robotPose = visionEstimate.estimatedPose.toPose2d();
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
    if (m_tagCount == 1 && m_ambiguity > VisionConstants.MAX_TAG_AMBIGUITY) {
      return;
    }
    // Dynamically scale measurement
    double xyStdDev = VisionConstants.BASE_XY_STD_DEV / m_tagCount;

    if (m_tagCount == 1 && !DriverStation.isAutonomous()) {
      xyStdDev *= 1.5;
    }
    double distanceScale = Math.pow(m_avgTagDistance, 2);

    xyStdDev *= distanceScale;

    Logger.recordOutput(m_logPath + "AcceptedPose", m_robotPose, Pose2d.struct);
    Logger.recordOutput(m_logPath + "XYStdDev", xyStdDev);

    m_drivetrain.addVisionMeasurement(
        m_robotPose, m_robotPoseTimestamp, VecBuilder.fill(xyStdDev, xyStdDev, 100000.0));
  }

  /**
   * Clears current vision state when no valid estimate is available.
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

  public boolean isConnected() {
    return m_camera.isConnected();
  }
}