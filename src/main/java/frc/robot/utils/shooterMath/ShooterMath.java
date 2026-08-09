package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;

public final class ShooterMath {
  /** Speed map for interpolating flywheel speeds. (Meters -> Rotations/Second) */
  private static final InterpolatingDoubleTreeMap m_speedMap = new InterpolatingDoubleTreeMap();

  /** Angle map for interpolating hood angles. (Meters -> Degrees) */
  private static final InterpolatingDoubleTreeMap m_angleMap = new InterpolatingDoubleTreeMap();

  /** Speed map for interpolating flywheel speeds. (Meters -> Rotations/Second) */
  private static final InterpolatingDoubleTreeMap m_speedPassMap = new InterpolatingDoubleTreeMap();

  /** Angle map for interpolating hood angles. (Meters -> Degrees) */
  private static final InterpolatingDoubleTreeMap m_anglePassMap = new InterpolatingDoubleTreeMap();

  static {
    // addToMaps(0, 35.5, 13.0);
    addToMaps(1.51, 47.0, 14.0); // Tune every .25m
    addToMaps(1.65, 47.5, 14.0);
    addToMaps(2.01, 50.0, 18.0);
    addToMaps(2.53, 53.0, 20.0);
    addToMaps(3.02, 56.5, 22.0);
    addToMaps(3.50, 60.0, 23.0);
    addToMaps(4.00, 63.25, 24.5);
    addToMaps(4.51, 70.0, 26.0);
    // addToMaps(100, 46.75, 13.0);

    addToPassMaps(4.82, 46.0, 50.0); // Tune every 0.1µ
    addToPassMaps(5.99, 53.0, 50.0);
    addToPassMaps(7.28, 62.0, 50.0);
    addToPassMaps(8.52, 71.0, 50.0);
    addToPassMaps(10.35, 82.0, 50.0);
    addToPassMaps(FieldConstants.kFieldLength.in(Meters), 82.0, 50.0); // Max distance possible

    SmartDashboard.putNumber("Tuning/ShooterMath/Angle", 0);
    SmartDashboard.putNumber("Tuning/ShooterMath/Speed", 0);
    SmartDashboard.putBoolean("Tuning/ShooterMath/TuningScore", false);
    SmartDashboard.putBoolean("Tuning/ShooterMath/TuningPass", false);
  }

  /** Maters, Rotations/Second, Degrees */
  private static void addToMaps(
      double distanceMeters, double flywheelRotPerS, double hoodAngleDegrees) {
    m_speedMap.put(distanceMeters, flywheelRotPerS);
    m_angleMap.put(distanceMeters, hoodAngleDegrees);
  }

  /** Maters, Rotations/Second, Degrees */
  private static void addToPassMaps(
      double distanceMeters, double flywheelRotPerS, double hoodAngleDegrees) {
    m_speedPassMap.put(distanceMeters, flywheelRotPerS);
    m_anglePassMap.put(distanceMeters, hoodAngleDegrees);
  }

  /**
   * Complete solution for a single shooter.
   *
   * @param robotHeading Robot heading to command for the shot.
   * @param hoodAngle Mechanical hood angle to command.
   * @param flywheelOmega Flywheel ω to command.
   */
  public record ShooterSolution(
      Rotation2d robotHeading, Angle hoodAngle, AngularVelocity flywheelOmega, Distance distance) {}

  /** Center of the hub */
  private static Translation2d m_targetHubCenter = new Translation2d();

  /** Center of the hub */
  private static Translation2d m_targetPassCenter = new Translation2d();

  /** The current solution for the shooter system. */
  public static ShooterSolution currentSolution =
      new ShooterSolution(Rotation2d.kZero, Degrees.of(0), RadiansPerSecond.of(0), Meters.of(0));

  /** The current passing solution for the shooter system. */
  public static ShooterSolution currentPassingSolution =
      new ShooterSolution(Rotation2d.kZero, Degrees.of(0), RadiansPerSecond.of(0), Meters.of(0));

  /** The current robot pose for the dual shooter system. */
  // Public so sim can access it
  public static Pose2d currentPose = new Pose2d();

  private static boolean m_morePow = false;

  private static double m_add = 0;

  /**
   * Compute optimal shooter parameters for a single shooter.
   *
   * @param robotPose Current robot 2-D pose.
   */
  public static void calculate(Pose2d robotPose) {
    currentPose = robotPose;
    m_targetHubCenter =
        (Alliance.redAlliance ? FieldConstants.kHubCenterRed : FieldConstants.kHubCenterBlue)
            .toTranslation2d();

    m_targetPassCenter =
        (Alliance.redAlliance
            ? (robotPose.getY() > FieldConstants.kFieldWidth.div(2).in(Meters)
                ? FieldConstants.kRedRightPassTarget
                : FieldConstants.kRedLeftPassTarget)
            : (robotPose.getY() < FieldConstants.kFieldWidth.div(2).in(Meters)
                ? FieldConstants.kBlueRightPassTarget
                : FieldConstants.kBlueLeftPassTarget));

    calculateScore(robotPose);
    calculatePass(robotPose);

    Robot.telemetry()
        .log("ShotSolution/Scoring/Heading", currentSolution.robotHeading, Rotation2d.struct);
    Robot.telemetry().log("ShotSolution/Scoring/Angle", currentSolution.hoodAngle);
    Robot.telemetry().log("ShotSolution/Scoring/FlywheelOmega", currentSolution.flywheelOmega);
    Robot.telemetry().log("ShotSolution/Scoring/Distance", currentSolution.distance);

    Robot.telemetry()
        .log(
            "ShotSolution/Passing/Heading", currentPassingSolution.robotHeading, Rotation2d.struct);
    Robot.telemetry().log("ShotSolution/Passing/Angle", currentPassingSolution.hoodAngle);
    Robot.telemetry()
        .log("ShotSolution/Passing/FlywheelOmega", currentPassingSolution.flywheelOmega);
    Robot.telemetry().log("ShotSolution/Passing/Distance", currentPassingSolution.distance);
  }

  public static void setMorePow(boolean morePow) {
    if (morePow != m_morePow) {
      m_add = morePow ? 2.0 : 0;
      calculate(currentPose);
      m_morePow = morePow;
    }
  }

  private static void calculateScore(Pose2d robotPose) {
    var dist = robotPose.getTranslation().getDistance(m_targetHubCenter);

    if (SmartDashboard.getBoolean("Tuning/ShooterMath/TuningScore", false)) {
      currentSolution =
          new ShooterSolution(
              m_targetHubCenter
                  .minus(robotPose.getTranslation())
                  .getAngle()
                  .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg),
              Degrees.of(SmartDashboard.getNumber("Tuning/ShooterMath/Angle", 0)),
              RotationsPerSecond.of(
                  SmartDashboard.getNumber("Tuning/ShooterMath/Speed", 0) + m_add),
              Meters.of(dist));
    } else {
      currentSolution =
          new ShooterSolution(
              m_targetHubCenter
                  .minus(robotPose.getTranslation())
                  .getAngle()
                  .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg),
              Degrees.of(m_angleMap.get(dist)),
              RotationsPerSecond.of(m_speedMap.get(dist) + m_add),
              Meters.of(dist));
    }
  }

  private static void calculatePass(Pose2d robotPose) {
    var dist = robotPose.getTranslation().getDistance(m_targetPassCenter);

    if (SmartDashboard.getBoolean("Tuning/ShooterMath/TuningPass", false)) {
      currentPassingSolution =
          new ShooterSolution(
              m_targetPassCenter
                  .minus(robotPose.getTranslation())
                  .getAngle()
                  .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg),
              Degrees.of(SmartDashboard.getNumber("Tuning/ShooterMath/Angle", 0)),
              RotationsPerSecond.of(
                  SmartDashboard.getNumber("Tuning/ShooterMath/Speed", 0) + m_add),
              Meters.of(dist));
    } else {
      currentPassingSolution =
          new ShooterSolution(
              m_targetPassCenter
                  .minus(robotPose.getTranslation())
                  .getAngle()
                  .plus(Alliance.redAlliance ? Rotation2d.kZero : Rotation2d.k180deg),
              Degrees.of(m_anglePassMap.get(dist)),
              RotationsPerSecond.of(m_speedPassMap.get(dist) + m_add),
              Meters.of(dist));
    }
  }

  public static ShooterSolution getScoreSolutionForDistance(Distance distance) {
    var dist = distance.in(Meters);
    return new ShooterSolution(
        Rotation2d.kZero,
        Degrees.of(m_angleMap.get(dist)),
        RotationsPerSecond.of(m_speedMap.get(dist)),
        distance);
  }
}
