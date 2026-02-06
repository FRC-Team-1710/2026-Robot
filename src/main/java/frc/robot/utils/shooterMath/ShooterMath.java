package frc.robot.utils.shooterMath;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.BooleanSupplier;

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
  private static final Translation2d kHUB_CENTER_BLUE = FieldConstants.kHubCenterBlue;

  // RED ALLIANCE
  private static final Translation2d kHUB_CENTER_RED = FieldConstants.kHubCenterRed;

  private static BooleanSupplier m_RedAlliance = () -> Alliance.redAlliance;

  // ===================== Class Variables =====================
  /* Input variables */
  private static Pose3d m_robotPose = new Pose3d();
  private static Velocity3d m_robotVelocity = new Velocity3d();

  /** Record representing the shoot state with velocity and angle. */
  public static record ShootState(AngularVelocity desiredRPM, Angle desiredAngle) {}

  /** Calculates the shooter velocity based on the horizontal distance to the target. */
  public static LinearVelocity findShooterVelocity(Pose3d robotPose) {
    // Translation2d to find the x and y of the robot on the field
    Translation2d robotTranslation = robotPose.getTranslation().toTranslation2d();

    double x_distance =
        !m_RedAlliance.getAsBoolean()
            ? robotTranslation.getDistance(kHUB_CENTER_BLUE)
            : robotTranslation.getDistance(kHUB_CENTER_RED);
    return MetersPerSecond.of(Math.sqrt((kA * x_distance * x_distance) / (kB * x_distance + kC)) + kD);
  }

  /** Calculates the shooter velocity based on the robot's pose and alliance color. */
  public static Velocity3d findShooterVelocity3d(Pose3d robotPose) {
    // Velocity3d that uses the distance and rotation to find the shooter velocity
    return new Velocity3d(findShooterVelocity(robotPose), robotPose.getRotation());
  }

  /** Calculates the shooter angle based on the horizontal distance to the target. */
  public static Angle findShooterAngle(Pose3d robotPose) {
    // Translation2d to find the x and y of the robot on the field
    Translation2d robotTranslation = robotPose.getTranslation().toTranslation2d();

    double x_distance =
        !m_RedAlliance.getAsBoolean()
            ? robotTranslation.getDistance(kHUB_CENTER_BLUE)
            : robotTranslation.getDistance(kHUB_CENTER_RED);
    return Radians.of(kE * Math.pow(kF, x_distance));
  }

  /**
   * Converts velocity in meters per second to RPM based on the shooter wheel diameter.
   *
   * @param vMetersPerSec Velocity in meters per second
   * @return Velocity in revolutions per minute
   */
  public static AngularVelocity velocityToRPM(LinearVelocity vMetersPerSec) {
    double circumference = Math.PI * ShooterConstants.WHEEL_DIAMETER;
    return RPM.of((vMetersPerSec.in(MetersPerSecond) / circumference)
        * 60.0); // Convert meters per second to revolutions per minute
  }

  /**
   * Calculates the shoot state (velocity and angle) based on the horizontal distance to the target.
   *
   * @param robotPose The current pose of the robot.
   * @return {@code ShootState} containing desired flywheel RPM and angle
   */
  public static ShootState calculateShootState(Pose3d robotPose, Velocity3d robotVelocity) {
    AngularVelocity desiredRPM = velocityToRPM(findShooterVelocity(robotPose));
    Angle desiredAngle = findShooterAngle(robotPose);
    return new ShootState(desiredRPM, desiredAngle);
  }

  /**
   * Calculates the shoot state (velocity and angle) based on the robot's current pose and alliance
   * color.
   *
   * @return {@code ShootState} containing desired flywheel RPM and angle
   */
  public static ShootState calculateShootState() {
    return calculateShootState(m_robotPose, m_robotVelocity);
  }

  /**
   * Inputs the robot's current pose for calculations.
   *
   * @param robotPose The current pose of the robot.
   */
  public static void input(Pose3d robotPose, Velocity3d robotVelocity) {
    m_robotPose = robotPose;
    m_robotVelocity = robotVelocity;
  }

  public static boolean getAlliance() {
    return !m_RedAlliance.getAsBoolean();
  }
}
