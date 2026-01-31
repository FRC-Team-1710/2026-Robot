package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.Alliance;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoPathBuilder {
  // Create a reusable builder with your robot's configuration
  public FollowPath.Builder pathBuilder;

  public static AutoPathBuilder instance;

  public static void setDrivetrainInstance(CommandSwerveDrivetrain drivetrain) {
    instance = new AutoPathBuilder();
    instance.pathBuilder =
        new FollowPath.Builder(
                drivetrain, // The drive subsystem to require
                drivetrain::getPose, // Supplier for current robot pose
                drivetrain::getRobotSpeeds, // Supplier for current speeds
                (speeds) ->
                    drivetrain.setControl(
                        drivetrain.bLineRequest.withSpeeds(speeds)), // Consumer to drive the robot
                new PIDController(5.0, 0.0, 0.0), // Translation PID
                new PIDController(3.0, 0.0, 0.0), // Rotation PID
                new PIDController(2.0, 0.0, 0.0) // Cross-track PID
                )
            .withShouldFlip(
                () ->
                    Alliance
                        .redAlliance); // Auto-flip for red alliance during auto, meaning that the
    // field is now identical to blue alliance for auto
    // .withPoseReset(drivetrain::resetPose); // Reset odometry at path start
  }

  public static FollowPath.Builder getBuilder() {
    return instance.pathBuilder;
  }
}
