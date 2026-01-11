package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import java.util.Optional;

/**
 * Limelight camera subsystem for tracking AprilTags and finding the robot's position.
 *
 * <p>This subsystem can:
 * <ul>
 *   <li>Figure out where the robot is using MegaTag (combines multiple AprilTag sightings)
 *   <li>Detect game pieces using the Limelight's AI detector
 *   <li>Account for camera lag to keep measurements accurate
 *   <li>Record all vision data for playback and debugging
 * </ul>
 *
 * <p><b>Setup:</b> You need to adjust camera height, angle, and other settings in VisionConstants
 * to match where the camera is mounted on your robot.
 *
 * <p><b>How it works:</b> The Limelight sends position data to the drivetrain, which uses it
 * to fix any drift in the wheel-based position tracking.
 *
 * @see LimelightHelpers
 */
@Logged
public class LimelightSubsystem extends SubsystemBase {
  private final String limelightName;
  private final CommandSwerveDrivetrain drivetrain;

  // Robot position from camera (using MegaTag)
  private Pose2d robotPose = new Pose2d(); // Where the camera thinks we are
  private double robotPoseTimestamp = 0.0; // When this measurement was taken
  private int tagCount = 0; // How many AprilTags the camera can see
  private double avgTagDistance = 0.0; // Average distance to visible tags (meters)
  private double ambiguity = 0.0; // Ambiguity value across all visible tags

  // System that records vision data for playback later
  private final HootAutoReplay autoReplay;

  public LimelightSubsystem(String limelightName, CommandSwerveDrivetrain drivetrain) {
    this.limelightName = limelightName;
    this.drivetrain = drivetrain;

    // Set up data recording for the Limelight
    this.autoReplay =
        new HootAutoReplay()
            .withStruct( // Record robot position
                "Limelight/" + limelightName + "/RobotPose",
                Pose2d.struct,
                () -> robotPose,
                val -> robotPose = val.value)
            .withDouble( // Record timestamp
                "Limelight/" + limelightName + "/RobotPoseTimestamp",
                () -> robotPoseTimestamp,
                val -> robotPoseTimestamp = val.value)
            .withInteger( // Record how many tags are visible
                "Limelight/" + limelightName + "/TagCount",
                () -> tagCount,
                val -> tagCount = val.value.intValue())
            .withDouble( // Record average tag distance
                "Limelight/" + limelightName + "/AvgTagDistance",
                () -> avgTagDistance,
                val -> avgTagDistance = val.value)
            .withDouble( // Record maximum ambiguity
                "Limelight/" + limelightName + "/Ambiguity",
                () -> ambiguity,
                val -> ambiguity = val.value).withTimestampReplay();
  }

  @Override
  public void periodic() {
    // Get new data from camera (unless we're replaying old data)
    if (!Utils.isReplay()) {
      fetchInputs();
    }

    // Update recording system (records on real robot, plays back during replay mode)
    autoReplay.update();

    // Use the camera data to update the robot's position
    processInputs();
  }

  private void fetchInputs() {
    // Get robot position estimate from Limelight (using blue alliance coordinates)
    LimelightHelpers.PoseEstimate poseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (poseEstimate != null && poseEstimate.tagCount > 0) {
      robotPose = poseEstimate.pose;
      robotPoseTimestamp = poseEstimate.timestampSeconds;
      tagCount = poseEstimate.tagCount;
      avgTagDistance = poseEstimate.avgTagDist;
      ambiguity = poseEstimate.rawFiducials[0].ambiguity;
    } else {
      robotPose = new Pose2d();
      robotPoseTimestamp = 0.0;
      tagCount = 0;
      avgTagDistance = 0.0;
      ambiguity = 0.0;
    }
  }

  private void processInputs() {
    // ==================== UPDATE ROBOT POSITION FROM APRILTAGS ====================
    // Send camera measurements to the drivetrain's position tracker
    if (tagCount > 0 && robotPoseTimestamp > 0) {
      // Check ambiguity before processing measurement. Ambiguity only happen on 1 tag
      if (tagCount == 1 && ambiguity > VisionConstants.MAX_TAG_AMBIGUITY) {
        return; // Reject measurement - don't add to drivetrain
      }

      // MegaTag combines camera and gyro data - camera for X/Y, gyro helps with rotation
      // Figure out how much to trust this measurement (more tags = more trust)
      double xyStdDev = VisionConstants.BASE_XY_STD_DEV / tagCount; // X/Y trust
      double thetaStdDev = VisionConstants.BASE_THETA_STD_DEV / tagCount; // Rotation trust

      // Trust the measurement less when tags are far away
      double avgDistDev = Math.pow(avgTagDistance,2);
      xyStdDev = xyStdDev * avgDistDev;
      thetaStdDev = thetaStdDev * avgDistDev;

      // Give the measurement to the drivetrain along with trust levels
      drivetrain.addVisionMeasurement(
          robotPose,
          robotPoseTimestamp,
          VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
    }
  }

  public Pose2d getRobotPose(){
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

  public double getAmbiguity(){
    return ambiguity;
  }
}
