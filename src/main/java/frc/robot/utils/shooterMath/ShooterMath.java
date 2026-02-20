package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
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

  /** Record representing the shoot state with velocity and angle. */
  public static record ShootState(AngularVelocity desiredRPM, Angle desiredAngle) {}

  // ===================== Class Variables =====================
  /* Input variables */
  private static Pose3d m_robotPose = new Pose3d();
  private static Velocity3d m_robotVelocity = new Velocity3d();

  /* Intermediate Variables */
  private static Velocity3d m_shootVelocity;

  /* Output Variables */
  private static ShootState m_shootState;
  private static Rotation2d m_robotRotation;

  /* INTERPOLATING TREE MAPS */
  private static InterpolatingDoubleTreeMap rpsMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();

  static { // Week 5 Saturday Testing
    // Adding values to the Interpolatable maps
    addInterpolatableValues(2.2, 50, 0.0874 * 360); // METERS, RPS, DEGREES
    addInterpolatableValues(2.5, 51.5, 0.1 * 360); // METERS, RPS, DEGREES
    addInterpolatableValues(3, 53, 0.11 * 360); // METERS, RPS, DEGREES
    addInterpolatableValues(3.5, 57.5, 0.138 * 360); // METERS, RPS, DEGREES
    addInterpolatableValues(4, 59, 0.15 * 360); // METERS, RPS, DEGREES
    addInterpolatableValues(5, 71.5, 0.156 * 360); // METERS, RPS, DEGREES
  }

  public static double calculateDistance() {
    Translation2d robotTranslation = m_robotPose.getTranslation().toTranslation2d();
    return !m_RedAlliance.getAsBoolean()
        ? robotTranslation.getDistance(kHUB_CENTER_BLUE)
        : robotTranslation.getDistance(kHUB_CENTER_RED);
  }

  /** Calculates the shooter velocity based on the horizontal distance to the target. */
  public static LinearVelocity findShooterVelocity() {
    double x_distance = calculateDistance();
    return MetersPerSecond.of(
        Math.sqrt((kA * x_distance * x_distance) / (kB * x_distance + kC)) + kD);
  }

  /** Calculates the shooter angle based on the horizontal distance to the target. */
  public static Angle findShooterAngle() {
    // Translation2d to find the x and y of the robot on the field
    Translation2d robotTranslation = m_robotPose.getTranslation().toTranslation2d();

    double x_distance =
        !m_RedAlliance.getAsBoolean()
            ? robotTranslation.getDistance(kHUB_CENTER_BLUE)
            : robotTranslation.getDistance(kHUB_CENTER_RED);
    return Radians.of(kE * Math.pow(kF, x_distance));
  }

  /** Calculates the shooter velocity based on the robot's pose and alliance color. */
  public static Velocity3d findShooterVelocity3d() {
    // Velocity3d that uses the distance and rotation to find the shooter velocity
    Translation2d hubPosition = !m_RedAlliance.getAsBoolean() ? kHUB_CENTER_BLUE : kHUB_CENTER_RED;
    Robot.telemetry()
        .log("HOLY COWS", m_robotPose.getTranslation().toTranslation2d().getDistance(hubPosition));
    Rotation2d hubAngle = m_robotPose.toPose2d().getTranslation().minus(hubPosition).getAngle();
    return new Velocity3d(
        findShooterVelocity(),
        new Rotation3d(0.0, findShooterAngle().in(Radians), hubAngle.getRadians()));
  }

  /**
   * Converts velocity in meters per second to RPM based on the shooter wheel diameter.
   *
   * @param vMetersPerSec Velocity in meters per second
   * @return Velocity in revolutions per minute
   */
  public static AngularVelocity velocityToRPM(LinearVelocity vMetersPerSec) {
    double circumference = Math.PI * ShooterConstants.WHEEL_DIAMETER;
    return RPM.of(
        (vMetersPerSec.in(MetersPerSecond) / circumference)
            * 60.0); // Convert meters per second to revolutions per minute
  }

  /**
   * Calculates the shoot state (velocity and angle) based on the horizontal distance to the target.
   *
   * @param robotPose The current pose of the robot.
   * @return {@code ShootState} containing desired flywheel RPM and angle
   */
  private static ShootState calculateShootState(Velocity3d shootVelocity) {
    AngularVelocity desiredRPM = velocityToRPM(MetersPerSecond.of(shootVelocity.norm()));
    Angle desiredAngle = Radians.of(Math.asin(shootVelocity.getZ() / shootVelocity.norm()));
    return new ShootState(desiredRPM, desiredAngle);
  }

  /**
   * Calculates the shoot state (velocity and angle) based on the robot's current pose and alliance
   * color.
   *
   * @return {@code ShootState} containing desired flywheel RPM and angle
   */
  public static void calculate() {
    m_shootVelocity = findShooterVelocity3d().plus(m_robotVelocity).inverse();
    m_shootState = calculateShootState(m_shootVelocity);
    m_robotRotation = new Rotation2d(m_shootVelocity.getX(), m_shootVelocity.getY());

    Robot.telemetry()
        .log(
            "ShooterMath",
            m_shootVelocity.toTranslation3d().plus(m_robotPose.getTranslation()),
            Translation3d.struct);
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

  /** Returns the current desired shooter RPM. */
  public static AngularVelocity getShooterRPM() {
    return m_shootState.desiredRPM();
  }

  /** Returns the current desired shooter angle. */
  public static Angle getShooterAngle() {
    return m_shootState.desiredAngle();
  }

  /** Returns the current pose of the robot. */
  public static Pose3d getRobotPose() {
    return m_robotPose;
  }

  /** Returns the current rotation of the robot. */
  public static Rotation2d getRobotRotation() {
    return m_robotRotation;
  }

  /* INTERPOLATING METHODS AND MATH */
  /** Adds a RPS value for the current robot pose distance. */
  public static void addRPS(double rps) {
    Translation2d robotTranslation = m_robotPose.getTranslation().toTranslation2d();

    double x_distance =
        !m_RedAlliance.getAsBoolean()
            ? robotTranslation.getDistance(kHUB_CENTER_BLUE)
            : robotTranslation.getDistance(kHUB_CENTER_RED);

    rpsMap.put(x_distance, rps);
  }

  /** Adds an angle value for the current robot pose distance. */
  public static void addAngle(double angle) {
    Translation2d robotTranslation = m_robotPose.getTranslation().toTranslation2d();

    double x_distance =
        !m_RedAlliance.getAsBoolean()
            ? robotTranslation.getDistance(kHUB_CENTER_BLUE)
            : robotTranslation.getDistance(kHUB_CENTER_RED);

    angleMap.put(x_distance, angle);
  }

  /** Gets the interpolated RPS based on the current robot pose distance. */
  public static double getInterpolatedRPS() {
    Translation2d robotTranslation = m_robotPose.getTranslation().toTranslation2d();

    double x_distance =
        !m_RedAlliance.getAsBoolean()
            ? robotTranslation.getDistance(kHUB_CENTER_BLUE)
            : robotTranslation.getDistance(kHUB_CENTER_RED);

    Robot.telemetry().log("X_DISTANCE", x_distance);
    Robot.telemetry().log("RPS", rpsMap.get(x_distance));
    Robot.telemetry().log("RadPS", RotationsPerSecond.of(rpsMap.get(x_distance)));

    return rpsMap.get(x_distance);
  }

  /** Gets the interpolated angle based on the current robot pose distance. */
  public static double getInterpolatedAngle() {
    Translation2d robotTranslation = m_robotPose.getTranslation().toTranslation2d();

    double x_distance =
        !m_RedAlliance.getAsBoolean()
            ? robotTranslation.getDistance(kHUB_CENTER_BLUE)
            : robotTranslation.getDistance(kHUB_CENTER_RED);

    return angleMap.get(x_distance);
  }

  public static void addInterpolatableValues(double distance, double rps, double angle) {
    rpsMap.put(distance, rps);
    angleMap.put(distance, angle);
  }
}
