package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;

@Logged
public class Vision extends SubsystemBase {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final CommandSwerveDrivetrain drivetrain;

  private Pose2d robotPose = new Pose2d();
  private double robotPoseTimestamp = 0.0;
  private int tagCount = 0;
  private double avgTagDistance = 0.0;
  private double ambiguity = 0.0;

  private final HootAutoReplay autoReplay;

  public Vision(String cameraName, Transform3d robotToCamera, CommandSwerveDrivetrain drivetrain) {

    this.drivetrain = drivetrain;

    camera = new PhotonCamera(cameraName);

    poseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTags, robotToCamera);

    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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

  @Override
  public void periodic() {

    if (!Utils.isReplay()) {
      fetchInputs();
    }

    autoReplay.update();
    processInputs();
  }

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

    if (tagCount == 1 && ambiguity > VisionConstants.MAX_TAG_AMBIGUITY) {
      return;
    }

    double xyStdDev = VisionConstants.BASE_XY_STD_DEV / tagCount;
    double thetaStdDev = VisionConstants.BASE_THETA_STD_DEV / tagCount;

    double distanceScale = Math.pow(avgTagDistance, 2);

    xyStdDev *= distanceScale;
    thetaStdDev *= distanceScale;

    drivetrain.addVisionMeasurement(
        robotPose, robotPoseTimestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }

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
