package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera m_camera;
  private final Transform3d m_robotToCamera;

  private final PhotonPoseEstimator m_photonEstimator;

  @SuppressWarnings("unused")
  private EstimatedRobotPose m_poseEstimate;

  private CommandSwerveDrivetrain drivetrain;

  private Matrix<N3, N1> m_curStdDevs;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.m_camera = new PhotonCamera(cameraName);
    this.m_robotToCamera = robotToCamera;

    this.m_photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, m_robotToCamera);
  }

  @Override
  public boolean isConnected() {
    return m_camera.isConnected();
  }

  @Override
  public PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  @Override
  public void process() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var result : m_camera.getAllUnreadResults()) {
      visionEst = m_photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = m_photonEstimator.estimateLowestAmbiguityPose(result);
      }
      updateEstimationStdDevs(visionEst, result.getTargets());
      if (drivetrain != null) {
        this.m_poseEstimate = visionEst.get();
        drivetrain.addVisionMeasurement(
            visionEst.get().estimatedPose.toPose2d(),
            visionEst.get().timestampSeconds,
            m_curStdDevs);
      }
    }
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      m_curStdDevs = VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        m_curStdDevs = VisionConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        m_curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return m_curStdDevs;
  }

  @Override
  public void setConsumer(CommandSwerveDrivetrain consumer) {
    this.drivetrain = consumer;
  }
}
