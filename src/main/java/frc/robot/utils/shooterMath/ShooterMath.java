package frc.robot.utils.shooterMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;

/** Utility class for shooter-related mathematical calculations. */
public class ShooterMath {

  // --- Hardware Specs ---
  private static final double kTopDiameter = 2.0;
  private static final double kBottomDiameter = 3.0;
  private static final double kCompression = 1.0;

  // --- Physics Constants ---
  private static final double kG = 9.80665;
  private static final double kTargetHeight = 72.0;

  // Backspin lift reduces required speed by ~12%
  private static final double kLiftCoefficient = 0.12;

  // --- Robot State ---
  private static Translation2d currentTranslation;
  private static ChassisSpeeds currentSpeeds;

  // Current solution
  private static ShotSolution currentSolution = new ShotSolution();

  public static class ShotSolution {
    public AngularVelocity flywheelSpeed;
    public Angle hoodAngle;
    public Rotation2d robotHeading;
  }

  public static ShotSolution calculate(
      double targetH, double relX, double relY, double robVx, double robVy) {

    double d = Math.sqrt(relX * relX + relY * relY);
    double gravityEffect = kG * (1.0 - kLiftCoefficient);

    // 1. Try for the "Ideal" Lowest RPM Angle (Apex Shot)
    double idealTheta = Math.atan((targetH + Math.sqrt(d * d + targetH * targetH)) / d);
    double idealThetaDeg = Math.toDegrees(idealTheta);

    double finalThetaRad;
    double requiredV0;
    boolean limited = false;

    // 2. Check Limits and Re-calculate if necessary
    if (idealThetaDeg < ShooterConstants.HOOD_MIN) {
      finalThetaRad = Math.toRadians(ShooterConstants.HOOD _MIN);
      requiredV0 = solveForFixedAngle(d, targetH, finalThetaRad, gravityEffect);
      limited = true;
    } else if (idealThetaDeg > MAX_HOOD_ANGLE) {
      finalThetaRad = Math.toRadians(MAX_HOOD_ANGLE);
      requiredV0 = solveForFixedAngle(d, targetH, finalThetaRad, gravityEffect);
      limited = true;
    } else {
      finalThetaRad = idealTheta;
      requiredV0 = Math.sqrt(gravityEffect * (targetH + Math.sqrt(d * d + targetH * targetH)));
    }

    // 3. Vector Compensation for Moving Robot
    double unitX = relX / d;
    double unitY = relY / d;
    double speedTowardTarget = (robVx * unitX) + (robVy * unitY);
    double adjustedV0 = requiredV0 + speedTowardTarget;

    // 4. RPM Calculation
    double v0InchesPerSec = adjustedV0 * 39.37;
    double dTopEff = WHEEL_TOP_DIA_IN - (COMPRESSION_IN / 2.0);
    double dBotEff = WHEEL_BOTTOM_DIA_IN - (COMPRESSION_IN / 2.0);

    double topRPM =
        (2.0 * v0InchesPerSec * 60.0) / (Math.PI * (dTopEff + (LINK_RATIO_B_TO_T * dBotEff)));

    // 5. Build Result
    ShotSolution sol = new ShotSolution();
    sol.motorRPM = topRPM * GEAR_RATIO_MOTOR_TO_TOP;
    sol.hoodAngleDeg = Math.toDegrees(finalThetaRad);
    sol.isLimited = limited;

    // Lateral Lead Aiming
    double lateralVel = (robVx * (-unitY)) + (robVy * unitX);
    double leadAngle = Math.atan2(lateralVel, requiredV0);
    sol.robotHeadingDeg = Math.toDegrees(Math.atan2(-relY, -relX) - leadAngle);

    return sol;
  }

  /** Calculates required velocity when the angle is fixed by physical limits. */
  private static double solveForFixedAngle(double d, double h, double theta, double g) {
    // Projectile motion formula solving for V0:
    // V0 = sqrt( (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - h)) )
    double numerator = g * Math.pow(d, 2);
    double denominator = 2 * Math.pow(Math.cos(theta), 2) * (d * Math.tan(theta) - h);

    if (denominator <= 0) return 0; // Target is physically unreachable at this angle
    return Math.sqrt(numerator / denominator);
  }

  // public static void calculate() {
  //   double d = currentTranslation.getNorm();

  //   double v0 =
  //       Math.sqrt(
  //           (kG * (1.0 - kLiftCoefficient))
  //               * (kTargetHeight + Math.sqrt(d * d + kTargetHeight * kTargetHeight)));

  //   double unitX = currentTranslation.getX() / d;
  //   double unitY = currentTranslation.getY() / d;

  //   currentSolution.hoodAngle =
  //       Radians.of(
  //           Math.atan((kTargetHeight + Math.sqrt(d * d + kTargetHeight * kTargetHeight)) / d));

  //   currentSolution.flywheelSpeed =
  //       RotationsPerSecond.of(
  //           (2.0
  //                   * (v0
  //                       + (currentSpeeds.vxMetersPerSecond * unitX)
  //                       + (currentSpeeds.vyMetersPerSecond * unitY))
  //                   * 39.37)
  //               / (Math.PI
  //                   * ((kTopDiameter - (kCompression / 2.0))
  //                       + (kBottomDiameter - (kCompression / 2.0)))));

  //   currentSolution.robotHeading =
  //       Rotation2d.fromRadians(
  //           Math.atan2(-currentTranslation.getY(), -currentTranslation.getX())
  //               - Math.atan2(
  //                   ((currentSpeeds.vxMetersPerSecond * (-unitY))
  //                       + (currentSpeeds.vyMetersPerSecond * unitX)),
  //                   v0));
  // }

  public static void updateRobotState(Translation2d translation, ChassisSpeeds speeds) {
    currentTranslation = translation;
    currentSpeeds = speeds;

    calculate();
  }

  public static ShotSolution getCurrentSolution() {
    return currentSolution;
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
