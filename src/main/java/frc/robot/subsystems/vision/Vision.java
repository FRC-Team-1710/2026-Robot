package frc.robot.subsystems.vision;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@Logged
public class Vision extends SubsystemBase {
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

  public Vision(String limelightName, CommandSwerveDrivetrain drivetrain) {
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
                val -> ambiguity = val.value)
            .withTimestampReplay();
  }

  @Override
  public void periodic() {
    if (!Utils.isReplay()) {
      fetchInputs();
    }

    // Update recording system (records on real robot, plays back during replay mode)
    autoReplay.update();

    // Use the camera data to update the robot's position
    processInputs();
  }

  private void fetchInputs() {

    // This entire thing doesn't work yo
    // TODO: Fix up the pose estimate stuff in phontonized helpers and then work on making all this
    // work. Import the correct thing

    /*
    // Get robot position estimate from Limelight (using blue alliance coordinates)
    LimelightHelpers.PoseEstimate poseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    // TODO: Replace LimelightHelpers functions with photonvision functions

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
      */
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
      double avgDistDev = Math.pow(avgTagDistance, 2);
      xyStdDev = xyStdDev * avgDistDev;
      thetaStdDev = thetaStdDev * avgDistDev;

      // Give the measurement to the drivetrain along with trust levels
      drivetrain.addVisionMeasurement(
          robotPose, robotPoseTimestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
    }
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
