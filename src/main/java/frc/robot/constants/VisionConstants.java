package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

  public static record PoseCameraConfig(String name, Transform3d robotToCamera) {}

  /**
   * Defines a single vision pose camera configuration.
   *
   * @param name The exact PhotonVision camera name (must match NetworkTables name)
   * @param robotToCamera Transform from robot center to camera lens in meters and radians.
   *     Translation3d: +X forward, +Y left, +Z up. Rotation3d: roll (X), pitch (Y), yaw (Z) in
   *     radians.
   */
  public static final PoseCameraConfig[] kPoseCameraConfigs = {
    new PoseCameraConfig(
        "FrontLeft",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.769),
                Units.inchesToMeters(13.341),
                Units.inchesToMeters(7.995)),
            new Rotation3d(Math.toRadians(180), Math.toRadians(180 + 45), Math.toRadians(175.0)))),
    new PoseCameraConfig(
        "FrontRight",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.769),
                Units.inchesToMeters(-13.341),
                Units.inchesToMeters(7.995)),
            new Rotation3d(Math.toRadians(180), Math.toRadians(180 + 45), Math.toRadians(185.0)))),
    new PoseCameraConfig(
        "BackLeft",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(9.61367),
                Units.inchesToMeters(16.48053),
                Units.inchesToMeters(28.975)),
            new Rotation3d(
                Math.toRadians(180),
                Math.toRadians(90.0),
                Math.toRadians(-90.0)))), // TODO: Change 0.0 pitch to 25 degrees in version 2
    new PoseCameraConfig(
        "BackRight",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(9.61367),
                Units.inchesToMeters(-16.48053),
                Units.inchesToMeters(28.975)),
            new Rotation3d(
                Math.toRadians(180),
                Math.toRadians(0.0),
                Math.toRadians(90.0)))), // TODO: Change 0.0 pitch to 25 degrees in version 2
  };

  public static record FuelCameraConfig(String name, Transform3d robotToCamera) {}

  public static final FuelCameraConfig[] kFuelCameraConfigs = {
    new FuelCameraConfig(
        "FuelCam",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(15.247),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(21.381)),
            new Rotation3d(
                0.0,
                Math.toRadians(0.0),
                Math.toRadians(90.0)))), // TODO: Change 0.0 pitch to 25 degrees in version 2  };
  };

  private VisionConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
