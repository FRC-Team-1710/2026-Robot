package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;

// Same as ShooterMath3 but built around the drum shooter redesign and is completely different
public final class ShooterMath4 {
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
    addToMaps(1.51, 47.0, 14.0);
    addToMaps(2.01, 49.0, 18.0);
    addToMaps(2.53, 53.0, 20.0);
    addToMaps(3.02, 56.5, 22.0);
    addToMaps(3.50, 59.75, 23.0);
    addToMaps(4.00, 63.25, 24.5);
    addToMaps(4.51, 68.0, 26.0);
    // addToMaps(100, 46.75, 13.0);

    addToPassMaps(0.0, 45.0, 40.0);
    addToPassMaps(1.51, 45.0, 40.0);
    addToPassMaps(2.24, 55.0, 43.0);
    addToPassMaps(3.67, 70.0, 46.0);
    addToPassMaps(5.36, 90.0, 47.0);

    SmartDashboard.putNumber("tuning/preferredMinArrivalAngleDeg", 0);
    SmartDashboard.putNumber("tuning/speedTransferEfficiency", 0);
    SmartDashboard.putBoolean("tuning/tuningShooter", false);
  }

  /** Maters, Rotations/Second, Degrees */
  private static void addToMaps(
      double distanceMeters, double flywheelRotPS, double hoodAngleDegrees) {
    m_speedMap.put(distanceMeters, flywheelRotPS);
    m_angleMap.put(distanceMeters, hoodAngleDegrees);
  }

  /** Maters, Rotations/Second, Degrees */
  private static void addToPassMaps(
      double distanceMeters, double flywheelRotPS, double hoodAngleDegrees) {
    m_speedPassMap.put(distanceMeters, flywheelRotPS);
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
      Rotation2d robotHeading, Angle hoodAngle, AngularVelocity flywheelOmega) {}

  /**
   * Complete solution for a single shooter.
   *
   * @param hoodAngle Mechanical hood angle to command.
   * @param flywheelOmega Flywheel ω to command.
   */
  public record SimpleSolution(Angle hoodAngle, AngularVelocity flywheelOmega) {}

  /** Center of the hub */
  private static Translation2d m_targetCenter = new Translation2d();

  /** The current solution for the shooter system. */
  public static ShooterSolution currentSolution =
      new ShooterSolution(Rotation2d.kZero, Degrees.of(0), RadiansPerSecond.of(0));

  /** The current passing solution for the shooter system. */
  public static SimpleSolution currentPassingSolution =
      new SimpleSolution(Degrees.of(0), RadiansPerSecond.of(0));

  /** The current robot pose for the dual shooter system. */
  // Public so sim can access it
  public static Pose2d currentPose = new Pose2d();

  /**
   * Compute optimal shooter parameters for a single shooter.
   *
   * @param robotPose Current robot 2-D pose.
   */
  public static void calculate(Pose2d robotPose) {
    currentPose = robotPose;
    m_targetCenter =
        (Alliance.redAlliance ? FieldConstants.kHubCenterRed : FieldConstants.kHubCenterBlue)
            .toTranslation2d();

    calculateComplexFromDirectionAndSpeed(robotPose);
    calculateSimpleFromDirectionAndSpeed(robotPose);

    if (Epilogue.shouldLog(Importance.INFO)) {
      Robot.telemetry()
          .log("ShotSolution/Scoring/Heading", currentSolution.robotHeading, Rotation2d.struct);
      Robot.telemetry().log("ShotSolution/Scoring/Angle", currentSolution.hoodAngle);
      Robot.telemetry().log("ShotSolution/Scoring/Omega", currentSolution.flywheelOmega);

      Robot.telemetry().log("ShotSolution/Passing/Angle", currentPassingSolution.hoodAngle);
      Robot.telemetry().log("ShotSolution/Passing/Omega", currentPassingSolution.flywheelOmega);
    }
  }

  private static void calculateComplexFromDirectionAndSpeed(Pose2d robotPose) {
    var dist = robotPose.getTranslation().getDistance(m_targetCenter);

    if (SmartDashboard.getBoolean("tuning/tuningShooter", false)) {
      currentSolution =
          new ShooterSolution(
              m_targetCenter.minus(robotPose.getTranslation()).getAngle(),
              Degrees.of(SmartDashboard.getNumber("tuning/preferredMinArrivalAngleDeg", 0)),
              RotationsPerSecond.of(SmartDashboard.getNumber("tuning/speedTransferEfficiency", 0)));
    } else {
      currentSolution =
          new ShooterSolution(
              m_targetCenter.minus(robotPose.getTranslation()).getAngle(),
              Degrees.of(m_angleMap.get(dist)),
              RotationsPerSecond.of(m_speedMap.get(dist)));
    }
  }

  private static void calculateSimpleFromDirectionAndSpeed(Pose2d robotPose) {
    var dist = Math.abs(robotPose.getX() - m_targetCenter.getX()) + Units.inchesToMeters(12);

    Robot.telemetry().log("SHOTDIST", dist);

    if (SmartDashboard.getBoolean("tuning/tuningShooter", false)) {
      currentPassingSolution =
          new SimpleSolution(
              Degrees.of(SmartDashboard.getNumber("tuning/preferredMinArrivalAngleDeg", 0)),
              RotationsPerSecond.of(SmartDashboard.getNumber("tuning/speedTransferEfficiency", 0)));
    } else {
      currentPassingSolution =
          new SimpleSolution(
              Degrees.of(m_anglePassMap.get(dist)),
              RotationsPerSecond.of(m_speedPassMap.get(dist)));
    }
  }
}
