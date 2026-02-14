package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Centralized configuration for all vision-related tuning and camera mounting geometry.
 *
 * <p>This class contains no logic
 */
public class VisionConstants {
  // ==================== Base Trust Levels ====================

  /** Starting trust level for X/Y position per AprilTag seen (meters) */
  public static final double BASE_XY_STD_DEV = 0.5;

  /** Starting trust level for rotation per AprilTag seen (radians) */
  public static final double BASE_THETA_STD_DEV = 5;

  // ==================== Ambiguity Filtering ====================

  /**
   * Maximum acceptable ambiguity for AprilTag detections (0.0 = perfect, 1.0 = completely
   * ambiguous)
   */
  public static final double MAX_TAG_AMBIGUITY = 0.7;

  // ==================== Trust Level Limits ====================

  /** Most we'll ever trust X/Y measurements (meters) - very confident */
  public static final double MIN_XY_STD_DEV = 0.01;

  /** Least we'll ever trust X/Y measurements (meters) - not confident */
  public static final double MAX_XY_STD_DEV = 2.0;

  /** Most we'll ever trust rotation measurements (radians) - very confident */
  public static final double MIN_THETA_STD_DEV = 0.05;

  /** Least we'll ever trust rotation measurements (radians) - not confident */
  public static final double MAX_THETA_STD_DEV = 1.0;

  public static record CameraConfig(String name, Transform3d robotToCamera) {}

  /**
   * Defines a single vision camera configuration.
   *
   * @param name The exact PhotonVision camera name (must match NetworkTables name)
   * @param robotToCamera Transform from robot center to camera lens in meters and radians.
   *     Translation3d: +X forward, +Y left, +Z up. Rotation3d: roll (X), pitch (Y), yaw (Z) in
   *     radians.
   */
  public static final CameraConfig[] kCameraConfigs = {
    // new CameraConfig("FrontLeft", new Transform3d(new Translation3d(), new Rotation3d())),
    // new CameraConfig("FrontRight", new Transform3d(new Translation3d(), new Rotation3d())),
    new CameraConfig(
        "BackLeft",
        new Transform3d(
            new Translation3d(9.61367, -16.48053, 28.975),
            new Rotation3d(
                0.0,
                Math.toRadians(0.0),
                Math.toRadians(-90.0)))), // TODO: Change 0.0 pitch to 25 degrees in version 2
    new CameraConfig(
        "BackRight",
        new Transform3d(
            new Translation3d(9.61367, 16.48053, 28.975),
            new Rotation3d(
                0.0,
                Math.toRadians(0.0),
                Math.toRadians(90.0)))), // TODO: Change 0.0 pitch to 25 degrees in version 2
    // new CameraConfig("FuelCam", new Transform3d(new Translation3d(), new Rotation3d())),
  };

  private VisionConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
