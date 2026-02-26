package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;

public final class ShooterMath2 {

  /** Ball diameter: 5.91 in → meters. */
  public static final double BALL_DIAMETER_M = 5.91 * 0.0254;

  /** Compression: 1 in → meters. */
  public static final double COMPRESSION_M = 1.00 * 0.0254;

  /** Bottom flywheel radius: 1.5 in → meters. */
  public static final double BOTTOM_WHEEL_RADIUS_M = 1.50 * 0.0254;

  /** Top flywheel radius: 1.0 in → meters. */
  public static final double TOP_WHEEL_RADIUS_M = 1.00 * 0.0254;

  /**
   * Distance from the bottom flywheel center to the ball center at the exit point (meters). The
   * ball center lies along the center-to-center line at:
   */
  public static final double EXIT_OFFSET_M =
      BOTTOM_WHEEL_RADIUS_M + (BALL_DIAMETER_M - COMPRESSION_M) / 2.0;

  /** Ball exit speed per unit flywheel. */
  private static final double SPEED_PER_OMEGA = (BOTTOM_WHEEL_RADIUS_M + TOP_WHEEL_RADIUS_M) / 2.0;

  // TODO:
  /** Minimum mechanical hood angle (radians) — steepest shot. */
  public static final double MIN_HOOD_ANGLE_RAD = Math.toRadians(ShooterConstants.HOOD_MIN);

  /** Maximum mechanical hood angle (radians) — flattest shot. */
  public static final double MAX_HOOD_ANGLE_RAD = Math.toRadians(ShooterConstants.HOOD_MAX);

  /** Gravitational acceleration (m/s²). */
  private static final double G = 9.80665;

  /** Maximum number of iterations for the solver. */
  private static final int MAX_ITER = 20;

  /** Time-of-flight epsilon for the solver (seconds). */
  private static final double TOF_EPSILON_S = 1e-7;

  /**
   * Physics solution for a single shooter at a fixed heading and TOF.
   *
   * @param robotHeading Field-frame robot heading for this shot.
   * @param hoodAngle Mechanical hood angle to command.
   * @param flywheelOmega Flywheel angular velocity to command.
   * @param hoodAngleClamped {@code true} if the optimal angle was outside the mechanical range.
   */
  public record ShooterSolution(
      Rotation2d robotHeading,
      Angle hoodAngle,
      AngularVelocity flywheelOmega,
      boolean hoodAngleClamped) {}

  /**
   * Combined solution for two fixed-direction shooters sharing one drive-base heading.
   *
   * @param robotHeading Field-frame heading to command to the drive base.
   * @param shooterLeft Solution for the first shooter (corresponds to {@code shooterLeft}).
   * @param shooterRight Solution for the second shooter (corresponds to {@code shooterRight}).
   */
  public record DualShooterSolution(
      Rotation2d robotHeading, ShooterSolution shooterLeft, ShooterSolution shooterRight) {}

  /** Center of the hub */
  private static Translation3d TARGET_CENTER = new Translation3d();

  /** The current solution for the dual shooter system. */
  public static DualShooterSolution currentSolution =
      new DualShooterSolution(
          Rotation2d.kZero,
          new ShooterSolution(Rotation2d.kZero, Radians.of(0), RadiansPerSecond.of(0), false),
          new ShooterSolution(Rotation2d.kZero, Radians.of(0), RadiansPerSecond.of(0), false));

  /** The current robot pose for the dual shooter system. */
  public static Pose2d currentPose = new Pose2d();

  /** The current speeds for the dual shooter system. */
  public static ChassisSpeeds currentSpeeds = new ChassisSpeeds();

  /**
   * Compute optimal shooter parameters for two fixed-direction shooters that share a single
   * drive-base heading.
   *
   * @param robotPose Current robot 2-D pose.
   * @param fieldSpeeds Chassis speeds in the field frame.
   */
  public static void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
    TARGET_CENTER =
        Alliance.redAlliance ? FieldConstants.kHubCenterRed : FieldConstants.kHubCenterBlue;
    currentPose = robotPose;
    currentSpeeds = fieldSpeeds;

    Translation3d pivot1 =
        new Pose3d(robotPose).transformBy(ShooterConstants.kLEFT_SHOOTER_OFFSET).getTranslation();
    Translation3d pivot2 =
        new Pose3d(robotPose).transformBy(ShooterConstants.kRIGHT_SHOOTER_OFFSET).getTranslation();

    SolverResult r1 = solveDirection(pivot1, fieldSpeeds);
    SolverResult r2 = solveDirection(pivot2, fieldSpeeds);

    // Distance-weighted circular mean of the two ideal directions.
    double D1 =
        Math.hypot(TARGET_CENTER.getX() - pivot1.getX(), TARGET_CENTER.getY() - pivot1.getY());
    double D2 =
        Math.hypot(TARGET_CENTER.getX() - pivot2.getX(), TARGET_CENTER.getY() - pivot2.getY());

    double totalW = D1 + D2;
    double phiComp =
        Math.atan2(
            (D1 * Math.sin(r1.phi) + D2 * Math.sin(r2.phi)) / totalW,
            (D1 * Math.cos(r1.phi) + D2 * Math.cos(r2.phi)) / totalW);

    Rotation2d robotHeading = new Rotation2d(phiComp);

    double tofGuess = (r1.tof + r2.tof) / 2.0;

    PhysicsResult phys1 = solvePhysicsAlongDirection(pivot1, fieldSpeeds, phiComp, tofGuess);
    PhysicsResult phys2 = solvePhysicsAlongDirection(pivot2, fieldSpeeds, phiComp, tofGuess);

    currentSolution =
        new DualShooterSolution(
            robotHeading,
            new ShooterSolution(
                robotHeading,
                Degrees.of(phys1.hoodAngle),
                RadiansPerSecond.of(phys1.exitSpeed / SPEED_PER_OMEGA),
                phys1.clamped),
            new ShooterSolution(
                robotHeading,
                Degrees.of(phys2.hoodAngle),
                RadiansPerSecond.of(phys2.exitSpeed / SPEED_PER_OMEGA),
                phys2.clamped));
  }

  /** Ideal field-frame shooting direction and converged TOF for one shooter. */
  private record SolverResult(double phi, double tof) {}

  /**
   * Iteratively solve for the ideal field-frame shooting direction (φ) and converged time-of-flight
   * for a shooter whose bottom flywheel center is at {@code pivotPos}.
   */
  private static SolverResult solveDirection(Translation3d pivotPos, ChassisSpeeds fieldSpeeds) {

    final double vRx = fieldSpeeds.vxMetersPerSecond;
    final double vRy = fieldSpeeds.vyMetersPerSecond;

    // Initial guess: ignore exit offset, shoot straight at target centre.
    double rawDx = TARGET_CENTER.getX() - pivotPos.getX();
    double rawDy = TARGET_CENTER.getY() - pivotPos.getY();
    double tof = Math.max(Math.hypot(rawDx, rawDy) / 8.0, 0.05);
    double phi = Math.atan2(rawDy, rawDx);

    // Seed hood angle estimate so the exit point can be computed on iter 0.
    double hoodAngle = (MIN_HOOD_ANGLE_RAD + MAX_HOOD_ANGLE_RAD) / 2.0;

    for (int iter = 0; iter < MAX_ITER; iter++) {

      // ---- Dynamic exit point ----
      // The ball exits along the center-to-center line at EXIT_OFFSET_M
      // from the pivot. The C-C line is at hoodAngle above horizontal and
      // points in the field-frame direction phi (the shooter faces phi).
      Translation3d exitPos = exitPoint(pivotPos, hoodAngle, phi);

      // Displacement from exit point to target, corrected for robot motion.
      double effDx = (TARGET_CENTER.getX() - exitPos.getX()) - vRx * tof;
      double effDy = (TARGET_CENTER.getY() - exitPos.getY()) - vRy * tof;
      double dz = TARGET_CENTER.getZ() - exitPos.getZ();
      double D = Math.hypot(effDx, effDy);

      if (D < 1e-4) {
        phi = Math.atan2(rawDy, rawDx);
        tof = 0.05;
        break;
      }

      phi = Math.atan2(effDy, effDx);

      // Optimal (unconstrained) launch angle → clamp to mechanical hood range.
      double optLaunch = Math.atan2(dz + Math.sqrt(dz * dz + D * D), D);
      hoodAngle = clamp(Math.PI / 2.0 - optLaunch, MIN_HOOD_ANGLE_RAD, MAX_HOOD_ANGLE_RAD);
      double launch = Math.PI / 2.0 - hoodAngle;

      double exitSpeed = minExitSpeed(D, dz, launch);
      if (!Double.isFinite(exitSpeed) || exitSpeed <= 0.0) {
        hoodAngle = (hoodAngle <= MIN_HOOD_ANGLE_RAD) ? MAX_HOOD_ANGLE_RAD : MIN_HOOD_ANGLE_RAD;
        launch = Math.PI / 2.0 - hoodAngle;
        exitSpeed = minExitSpeed(D, dz, launch);
      }

      double cosL = Math.cos(launch);
      double newTof =
          (Double.isFinite(exitSpeed) && exitSpeed > 0 && cosL > 1e-9)
              ? D / (exitSpeed * cosL)
              : tof;

      if (!Double.isFinite(newTof) || newTof <= 0.0) newTof = tof;

      if (Math.abs(newTof - tof) < TOF_EPSILON_S) {
        tof = newTof;
        break;
      }
      tof = newTof;
    }

    return new SolverResult(phi, tof);
  }

  /** Per-shooter physics result after the shared heading is fixed. */
  private record PhysicsResult(double hoodAngle, double exitSpeed, boolean clamped) {}

  /**
   * Given a fixed field-frame shooting direction φ (heading decided), solve for the hood angle and
   * exit speed for the shooter whose bottom flywheel center is at {@code pivotPos}.
   *
   * @param pivotPos Bottom flywheel center in field frame.
   * @param fieldSpeeds Field-frame chassis speeds.
   * @param phi Fixed field-frame shooting direction (radians).
   * @param tofGuess Initial TOF estimate from the direction solve (seconds).
   */
  private static PhysicsResult solvePhysicsAlongDirection(
      Translation3d pivotPos, ChassisSpeeds fieldSpeeds, double phi, double tofGuess) {

    final double vRx = fieldSpeeds.vxMetersPerSecond;
    final double vRy = fieldSpeeds.vyMetersPerSecond;
    final double cosPhi = Math.cos(phi);
    final double sinPhi = Math.sin(phi);

    double tof = Math.max(tofGuess, 0.05);
    double hood = (MIN_HOOD_ANGLE_RAD + MAX_HOOD_ANGLE_RAD) / 2.0;
    double speed = 0.0;
    boolean clamped = false;

    for (int iter = 0; iter < MAX_ITER; iter++) {

      // Dynamic exit point at current hood estimate
      Translation3d exitPos = exitPoint(pivotPos, hood, phi);

      double effDx = (TARGET_CENTER.getX() - exitPos.getX()) - vRx * tof;
      double effDy = (TARGET_CENTER.getY() - exitPos.getY()) - vRy * tof;
      double dz = TARGET_CENTER.getZ() - exitPos.getZ();

      // Project onto shooting direction for the along-barrel distance.
      double D = effDx * cosPhi + effDy * sinPhi;

      if (D < 1e-4) {
        hood = MAX_HOOD_ANGLE_RAD;
        clamped = true;
        double la = Math.PI / 2.0 - hood;
        speed = minExitSpeed(1e-4, dz, la);
        tof = 1e-4 / (speed * Math.cos(la));
        break;
      }

      double optLaunch = Math.atan2(dz + Math.sqrt(dz * dz + D * D), D);
      double optHood = Math.PI / 2.0 - optLaunch;
      clamped = (optHood < MIN_HOOD_ANGLE_RAD || optHood > MAX_HOOD_ANGLE_RAD);
      hood = clamp(optHood, MIN_HOOD_ANGLE_RAD, MAX_HOOD_ANGLE_RAD);

      double launch = Math.PI / 2.0 - hood;
      speed = minExitSpeed(D, dz, launch);

      if (!Double.isFinite(speed) || speed <= 0.0) {
        hood = (hood <= MIN_HOOD_ANGLE_RAD) ? MAX_HOOD_ANGLE_RAD : MIN_HOOD_ANGLE_RAD;
        clamped = true;
        launch = Math.PI / 2.0 - hood;
        speed = minExitSpeed(D, dz, launch);
      }

      double cosL = Math.cos(launch);
      double newTof =
          (Double.isFinite(speed) && speed > 0 && cosL > 1e-9) ? D / (speed * cosL) : tof;

      if (!Double.isFinite(newTof) || newTof <= 0.0) newTof = tof;

      if (Math.abs(newTof - tof) < TOF_EPSILON_S) {
        tof = newTof;
        break;
      }
      tof = newTof;
    }

    return new PhysicsResult(hood, speed, clamped);
  }

  /**
   * Computes the ball exit point in field frame given the bottom flywheel center position, the
   * current hood angle, and the field-frame shooting direction.
   *
   * @param pivotPos Bottom flywheel center in field frame.
   * @param hoodAngle Mechanical hood angle (radians above horizontal).
   * @param phi Field-frame shooting direction (yaw, radians).
   * @return Ball exit point in field frame.
   */
  private static Translation3d exitPoint(Translation3d pivotPos, double hoodAngle, double phi) {

    double cosHood = Math.cos(hoodAngle);
    double sinHood = Math.sin(hoodAngle);

    return new Translation3d(
        pivotPos.getX() + EXIT_OFFSET_M * cosHood * Math.cos(phi),
        pivotPos.getY() + EXIT_OFFSET_M * cosHood * Math.sin(phi),
        pivotPos.getZ() + EXIT_OFFSET_M * sinHood);
  }

  /**
   * Minimum exit speed (m/s) for horizontal distance D, vertical rise dz, and launch angle theta.
   *
   * @return m/s, or {@link Double#NaN} if unreachable at this angle.
   */
  private static double minExitSpeed(double D, double dz, double theta) {
    double cosT = Math.cos(theta);
    double den = 2.0 * cosT * cosT * (D * Math.tan(theta) - dz);
    return (den > 0.0) ? Math.sqrt(G * D * D / den) : Double.NaN;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private ShooterMath2() {}
}
