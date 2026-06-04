package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

// Same as ShooterMath3 but built around the drum shooter redesign and is completely different
public final class ShooterMath4 {
  /** Speed map for interpolating flywheel speeds. (Meters -> Rotations/Second) */
  private static final InterpolatingDoubleTreeMap m_speedMap = new InterpolatingDoubleTreeMap();

  /** Angle map for interpolating hood angles. (Meters -> Degrees) */
  private static final InterpolatingDoubleTreeMap m_angleMap = new InterpolatingDoubleTreeMap();

  /** Speed map for interpolating flywheel speeds. (Meters -> Rotations/Second) */
  private static final InterpolatingDoubleTreeMap m_speedBumpMap = new InterpolatingDoubleTreeMap();

  /** Angle map for interpolating hood angles. (Meters -> Degrees) */
  private static final InterpolatingDoubleTreeMap m_angleBumpMap = new InterpolatingDoubleTreeMap();

  /** Speed map for interpolating flywheel speeds. (Meters -> Rotations/Second) */
  private static final InterpolatingDoubleTreeMap m_speedTrenchMap =
      new InterpolatingDoubleTreeMap();

  /** Angle map for interpolating hood angles. (Meters -> Degrees) */
  private static final InterpolatingDoubleTreeMap m_angleTrenchMap =
      new InterpolatingDoubleTreeMap();

  static { // Need to retune for every 0.25 meters (1678 did this same thing)
    // addToMaps(0, 35.5, 13.0);
    addToMaps(1.51, 47.0, 14.0);
    addToMaps(1.65, 47.5, 14.0);
    addToMaps(2.01, 50.0, 18.0);
    addToMaps(2.53, 53.0, 20.0);
    addToMaps(3.02, 56.5, 22.0);
    addToMaps(3.50, 59.75, 23.0);
    addToMaps(4.00, 63.25, 24.5);
    addToMaps(4.51, 68.0, 26.0);
    // addToMaps(100, 46.75, 13.0);

    addToBumpMaps(0.0, 45.0, 40.0);
    addToBumpMaps(1.51, 45.0, 40.0);
    addToBumpMaps(2.24, 55.0, 43.0);
    addToBumpMaps(3.67, 70.0, 46.0);
    addToBumpMaps(5.36, 90.0, 47.0);

    addToTrenchMaps(0.0, 45.0, 40.0);
    addToTrenchMaps(1.51, 45.0, 40.0);
    addToTrenchMaps(2.24, 55.0, 43.0);
    addToTrenchMaps(3.67, 70.0, 46.0);
    addToTrenchMaps(5.36, 90.0, 47.0);

    SmartDashboard.putNumber("Tuning/ShooterMath4/HoodAngleDeg", 0);
    SmartDashboard.putNumber("Tuning/ShooterMath4/FlywheelOmegaRotPS", 0);
    SmartDashboard.putBoolean("Tuning/ShooterMath4/Tuning", false);
  }

  /** Maters, Rotations/Second, Degrees */
  private static void addToMaps(
      double distanceMeters, double flywheelRotPS, double hoodAngleDegrees) {
    m_speedMap.put(distanceMeters, flywheelRotPS);
    m_angleMap.put(distanceMeters, hoodAngleDegrees);
  }

  /** Maters, Rotations/Second, Degrees */
  private static void addToBumpMaps(
      double distanceMeters, double flywheelRotPS, double hoodAngleDegrees) {
    m_speedBumpMap.put(distanceMeters, flywheelRotPS);
    m_angleBumpMap.put(distanceMeters, hoodAngleDegrees);
  }

  /** Maters, Rotations/Second, Degrees */
  private static void addToTrenchMaps(
      double distanceMeters, double flywheelRotPS, double hoodAngleDegrees) {
    m_speedTrenchMap.put(distanceMeters, flywheelRotPS);
    m_angleTrenchMap.put(distanceMeters, hoodAngleDegrees);
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

  /** Center of the hub */
  private static Translation2d m_targetHubCenter = new Translation2d();

  /** Target Tump */
  private static FieldConstants.Tumps.BlueTump m_targetTump =
      FieldConstants.Tumps.BlueTump.BLUE_LEFT_BUMP;

  /** The current solution for the shooter system. */
  public static ShooterSolution currentSolution =
      new ShooterSolution(Rotation2d.kZero, Degrees.of(0), RadiansPerSecond.of(0));

  /** The current passing solution for the shooter system. */
  public static ShooterSolution currentPassingSolution =
      new ShooterSolution(Rotation2d.kZero, Degrees.of(0), RadiansPerSecond.of(0));

  // Public so fuel sim can access it
  /** The current robot pose for the dual shooter system. */
  public static Pose2d currentPose = new Pose2d();

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

    m_targetTump =
        FieldConstants.Tumps.BlueTump.getClosest(
            robotPose
                .getTranslation()
                .rotateAround(FieldConstants.kFieldCenter, Rotation2d.k180deg));

    calculateScore(robotPose);
    calculatePass(robotPose);

    Logger.recordOutput("ShooterMath4/Scoring/Heading", currentSolution.robotHeading);
    Logger.recordOutput("ShooterMath4/Scoring/Angle", currentSolution.hoodAngle);
    Logger.recordOutput("ShooterMath4/Scoring/Omega", currentSolution.flywheelOmega);

    Logger.recordOutput("ShooterMath4/Passing/Heading", currentPassingSolution.robotHeading);
    Logger.recordOutput("ShooterMath4/Passing/Angle", currentPassingSolution.hoodAngle);
    Logger.recordOutput("ShooterMath4/Passing/Omega", currentPassingSolution.flywheelOmega);

    Logger.recordOutput("ShooterMath4/Passing/TargetTump", m_targetTump);
  }

  private static void calculateScore(Pose2d robotPose) {
    var dist = robotPose.getTranslation().getDistance(m_targetHubCenter);

    Logger.recordOutput("ShooterMath4/Scoring/Distance", dist);

    if (SmartDashboard.getBoolean("Tuning/ShooterMath4/Tuning", false)) {
      currentSolution =
          new ShooterSolution(
              m_targetHubCenter.minus(robotPose.getTranslation()).getAngle(),
              Degrees.of(SmartDashboard.getNumber("Tuning/ShooterMath4/HoodAngleDeg", 0)),
              RotationsPerSecond.of(
                  SmartDashboard.getNumber("Tuning/ShooterMath4/FlywheelOmegaRotPS", 0)));
    } else {
      currentSolution =
          new ShooterSolution(
              m_targetHubCenter.minus(robotPose.getTranslation()).getAngle(),
              Degrees.of(m_angleMap.get(dist)),
              RotationsPerSecond.of(m_speedMap.get(dist)));
    }
  }

  private static void calculatePass(Pose2d robotPose) {
    var dist = robotPose.getTranslation().getDistance(m_targetTump.average);

    Logger.recordOutput("ShooterMath4/Passing/Distance", dist);

    if (m_targetTump.equals(FieldConstants.Tumps.BlueTump.BLUE_LEFT_BUMP)
        || m_targetTump.equals(FieldConstants.Tumps.BlueTump.BLUE_RIGHT_BUMP)) {
      if (SmartDashboard.getBoolean("Tuning/ShooterMath4/Tuning", false)) {
        currentPassingSolution =
            new ShooterSolution(
                (Alliance.redAlliance
                        ? robotPose
                            .getTranslation()
                            .rotateAround(FieldConstants.kFieldCenter, Rotation2d.k180deg)
                        : robotPose.getTranslation())
                    .minus(m_targetTump.average)
                    .getAngle(),
                Degrees.of(SmartDashboard.getNumber("Tuning/ShooterMath4/HoodAngleDeg", 0)),
                RotationsPerSecond.of(
                    SmartDashboard.getNumber("Tuning/ShooterMath4/FlywheelOmegaRotPS", 0)));
      } else {
        currentPassingSolution =
            new ShooterSolution(
                (Alliance.redAlliance
                        ? robotPose
                            .getTranslation()
                            .rotateAround(FieldConstants.kFieldCenter, Rotation2d.k180deg)
                        : robotPose.getTranslation())
                    .minus(m_targetTump.average)
                    .getAngle(),
                Degrees.of(m_angleBumpMap.get(dist)),
                RotationsPerSecond.of(m_speedBumpMap.get(dist)));
      }
    } else {
      if (SmartDashboard.getBoolean("Tuning/ShooterMath4/Tuning", false)) {
        currentPassingSolution =
            new ShooterSolution(
                (Alliance.redAlliance
                        ? robotPose
                            .getTranslation()
                            .rotateAround(FieldConstants.kFieldCenter, Rotation2d.k180deg)
                        : robotPose.getTranslation())
                    .minus(m_targetTump.average)
                    .getAngle(),
                Degrees.of(SmartDashboard.getNumber("Tuning/ShooterMath4/HoodAngleDeg", 0)),
                RotationsPerSecond.of(
                    SmartDashboard.getNumber("Tuning/ShooterMath4/FlywheelOmegaRotPS", 0)));
      } else {
        currentPassingSolution =
            new ShooterSolution(
                (Alliance.redAlliance
                        ? robotPose
                            .getTranslation()
                            .rotateAround(FieldConstants.kFieldCenter, Rotation2d.k180deg)
                        : robotPose.getTranslation())
                    .minus(m_targetTump.average)
                    .getAngle(),
                Degrees.of(m_angleTrenchMap.get(dist)),
                RotationsPerSecond.of(m_speedTrenchMap.get(dist)));
      }
    }
  }
}
