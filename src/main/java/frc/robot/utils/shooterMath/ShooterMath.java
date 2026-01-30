package frc.robot.utils.shooterMath;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.ShooterConstants;

/** Utility class for shooter-related mathematical calculations. */
public class ShooterMath {
  // ================== Shooter Math Constants ========================

  /* See Desmos for the shooter math calculations: https://www.desmos.com/calculator/sgkhoo58x4 */
  // Velocity Function Constants (sqrt((kA*x^2)/(kB*x + kC)) + kD)
  private static final double kA = 2.68496;
  private static final double kB = 0.11359;
  private static final double kC = 0.0680981;
  private static final double kD = 0.746731;

  // Angle Function Constants (kE * kF ^ x)
  private static final double kE = 1.60189;
  private static final double kF = 0.889922;

  /** Position of the hub's center */
  // BLUE ALLIANCE
  private static final Translation2d HUB_CENTER_BLUE = new Translation2d(0, 0);

  // RED ALLIANCE
  private static final Translation2d HUB_CENTER_RED = new Translation2d(0, 0);

  /** Record representing the shoot state with velocity and angle. */
  public record ShootState(double velocity, double angle) {}

  /** Calculates the shooter velocity based on the horizontal distance to the target. */
  public static double findShooterVelocity(Pose3d robotPose, boolean blueAlliance) {
    // Translation2d to find the x and y of the robot on the field
    Translation2d robotTranslation = robotPose.getTranslation().toTranslation2d();

    double x_distance =
        blueAlliance
            ? robotTranslation.getDistance(HUB_CENTER_BLUE)
            : robotTranslation.getDistance(HUB_CENTER_RED);
    return Math.sqrt((kA * x_distance * x_distance) / (kB * x_distance + kC)) + kD;
  }

  /** Calculates the shooter velocity based on the robot's pose and alliance color. */
  public static Velocity3d findShooterVelocity3d(Pose3d robotPose, boolean blueAlliance) {
    // Velocity3d that uses the distance and rotation to find the shooter velocity
    return new Velocity3d(findShooterVelocity(robotPose, blueAlliance), robotPose.getRotation());
  }

  /** Calculates the shooter angle based on the horizontal distance to the target. */
  public static double findShooterAngle(Pose3d robotPose, boolean blueAlliance) {
    // Translation2d to find the x and y of the robot on the field
    Translation2d robotTranslation = robotPose.getTranslation().toTranslation2d();

    double x_distance =
        blueAlliance
            ? robotTranslation.getDistance(HUB_CENTER_BLUE)
            : robotTranslation.getDistance(HUB_CENTER_RED);
    return kE * Math.pow(kF, x_distance);
  }

  /** Converts velocity in meters per second to RPM based on the shooter wheel diameter. */
  public static double velocityToRPM(double vMetersPerSec) {
    double circumference = Math.PI * ShooterConstants.WHEEL_DIAMETER;
    return (vMetersPerSec / circumference) * 60.0;
  }

  /**
   * Calculates the shoot state (velocity and angle) based on the horizontal distance to the target.
   */
  public static ShootState calculateShootState(Pose3d robotPose, boolean blueAlliance) {
    double velocity = findShooterVelocity(robotPose, blueAlliance);
    double angle = findShooterAngle(robotPose, blueAlliance);
    return new ShootState(velocity, angle);
  }
}
