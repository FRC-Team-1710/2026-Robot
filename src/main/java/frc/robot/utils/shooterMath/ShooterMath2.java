package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;

public final class ShooterMath2 {
  /**
   * Preferred minimum arrival angle (radians). 0° = Disabled, 30° = Only activates at long range,
   * 45° = Ball enters at roughly 45° into the funnel, 60° = Noticeably higher flywheel demand at
   * mid-range, 70° = May be unreachable from far positions
   */
  // Soon to be k, need to tune first
  private static double m_preferredMinArrivalAngleRad = Math.toRadians(45.0);

  /** Efficiency of speed transfer from flywheel to ball. */
  // Soon to be k, need to tune first
  private static double m_speedTransferEfficiency = 0.5; // 1.0 = No slip

  /** Ball diameter: 5.91 in → meters. */
  private static final double kBallDiameterMeters = 5.91 * 0.0254;

  /** Ball radius: 2.955 in → meters. */
  private static final double kBallRadiusMeters = kBallDiameterMeters / 2.0;

  /** Compression: 1 in → meters. */
  private static final double kCompressionMeters = 1.00 * 0.0254;

  /** Bottom flywheel radius: 1.5 in → meters. */
  private static final double kBottomWheelRadiusMeters = 1.50 * 0.0254;

  /** Top flywheel radius: 1.0 in → meters. */
  private static final double kTopWheelRadiusMeters = 1.00 * 0.0254;

  /**
   * Distance from the bottom flywheel center to the ball center at the exit point (meters). The
   * ball center lies along the center-to-center line at:
   */
  private static final double kExitOffsetMeters =
      kBottomWheelRadiusMeters + (kBallDiameterMeters - kCompressionMeters) / 2.0;

  /** Ball exit speed per unit flywheel. */
  private static double kSpeedPerOmega =
      m_speedTransferEfficiency * (kBottomWheelRadiusMeters + kTopWheelRadiusMeters) / 2.0;

  /** Minimum mechanical hood angle (radians) — steepest shot. */
  private static final double kMinHoodAngleRad = Math.toRadians(ShooterConstants.HOOD_MIN);

  /** Maximum mechanical hood angle (radians) — flattest shot. */
  private static final double kMaxHoodAngleRad = Math.toRadians(ShooterConstants.HOOD_MAX);

  /** Gravitational acceleration (m/s²). */
  private static final double kG = 9.80665;

  /** Maximum number of iterations for the solver. */
  private static final int kMaxIter = 20;

  /** Time-of-flight epsilon for the solver (seconds). */
  private static final double kTofEpsilonSeconds = 1e-7;

  /**
   * Complete solution for a single shooter.
   *
   * @param hoodAngle Mechanical hood angle to command.
   * @param flywheelOmega Flywheel ω to command.
   * @param tof Time of flight for the shot.
   * @param hoodAngleClamped Optimal angle was outside the mechanical range.
   * @param arrivalAngleConstrained Shot was made steeper than minimum-velocity optimum to clear the
   *     near edge of the funnel.
   * @param tolerances Per-parameter deviation budgets for this shot.
   */
  public record ShooterSolution(
      Angle hoodAngle,
      AngularVelocity flywheelOmega,
      Time tof,
      boolean hoodAngleClamped,
      boolean arrivalAngleConstrained) {}

  /**
   * Combined solution for two fixed-direction shooters sharing one drive-base heading.
   *
   * @param robotHeading Field-frame heading to command.
   * @param shooterLeft Solution for the left shooter ({@code robotToShooter1}).
   * @param shooterRight Solution for the right shooter ({@code robotToShooter2}).
   */
  public record DualShooterSolution(
      Rotation2d robotHeading, ShooterSolution shooterLeft, ShooterSolution shooterRight) {}

  /** Center of the hub */
  private static Translation3d m_targetCenter = new Translation3d();

  /** Center of the hub */
  private static Translation3d[] m_targetVertices = new Translation3d[0];

  /** The current solution for the dual shooter system. */
  public static DualShooterSolution currentSolution =
      new DualShooterSolution(
          Rotation2d.kZero,
          new ShooterSolution(Radians.of(0), RadiansPerSecond.of(0), Seconds.of(1), false, false),
          new ShooterSolution(Radians.of(0), RadiansPerSecond.of(0), Seconds.of(1), false, false));

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
    m_preferredMinArrivalAngleRad =
        Math.toRadians(SmartDashboard.getNumber("preferredMinArrivalAngleDeg", 0));
    m_speedTransferEfficiency = SmartDashboard.getNumber("speedTransferEfficiency", 0);
    kSpeedPerOmega =
        m_speedTransferEfficiency * (kBottomWheelRadiusMeters + kTopWheelRadiusMeters) / 2.0;

    m_targetCenter =
        Alliance.redAlliance ? FieldConstants.kHubCenterRed : FieldConstants.kHubCenterBlue;
    m_targetVertices =
        Alliance.redAlliance ? FieldConstants.kHexagonRed : FieldConstants.kHexagonBlue;
    currentPose = robotPose;
    currentSpeeds = fieldSpeeds;

    Translation3d pivot1 =
        new Pose3d(robotPose).transformBy(ShooterConstants.kLEFT_SHOOTER_OFFSET).getTranslation();
    Translation3d pivot2 =
        new Pose3d(robotPose).transformBy(ShooterConstants.kRIGHT_SHOOTER_OFFSET).getTranslation();

    SolverResult r1 = solveDirection(pivot1, fieldSpeeds);
    SolverResult r2 = solveDirection(pivot2, fieldSpeeds);

    double D1 =
        Math.hypot(m_targetCenter.getX() - pivot1.getX(), m_targetCenter.getY() - pivot1.getY());
    double D2 =
        Math.hypot(m_targetCenter.getX() - pivot2.getX(), m_targetCenter.getY() - pivot2.getY());

    double phiComp =
        Math.atan2(
            (D1 * Math.sin(r1.phi) + D2 * Math.sin(r2.phi)) / (D1 + D2),
            (D1 * Math.cos(r1.phi) + D2 * Math.cos(r2.phi)) / (D1 + D2));

    double tofGuess = (r1.tof + r2.tof) / 2.0;

    PhysicsResult phys1 = solvePhysicsAlongDirection(pivot1, fieldSpeeds, phiComp, tofGuess);
    PhysicsResult phys2 = solvePhysicsAlongDirection(pivot2, fieldSpeeds, phiComp, tofGuess);

    double omega1 = phys1.exitSpeed / kSpeedPerOmega;
    double omega2 = phys2.exitSpeed / kSpeedPerOmega;

    currentSolution =
        new DualShooterSolution(
            new Rotation2d(phiComp),
            new ShooterSolution(
                Radians.of(phys1.hoodAngle),
                RadiansPerSecond.of(omega1),
                Seconds.of(phys1.tof),
                phys1.hoodClamped,
                phys1.arrivalConstrained),
            new ShooterSolution(
                Radians.of(phys2.hoodAngle),
                RadiansPerSecond.of(omega2),
                Seconds.of(phys2.tof),
                phys2.hoodClamped,
                phys2.arrivalConstrained));

    if (Epilogue.shouldLog(Importance.INFO)) {
      Robot.telemetry()
          .log("ShotSolution/Heading", currentSolution.robotHeading, Rotation2d.struct);

      Robot.telemetry().log("ShotSolution/Left/Angle", currentSolution.shooterLeft.hoodAngle);
      Robot.telemetry().log("ShotSolution/Left/Omega", currentSolution.shooterLeft.flywheelOmega);
      Robot.telemetry().log("ShotSolution/Left/TOF", currentSolution.shooterLeft.tof);
      Robot.telemetry()
          .log("ShotSolution/Left/Clamped", currentSolution.shooterLeft.hoodAngleClamped);
      Robot.telemetry()
          .log(
              "ShotSolution/Left/ArrivalConstraint",
              currentSolution.shooterLeft.arrivalAngleConstrained);

      Robot.telemetry().log("ShotSolution/Right/Angle", currentSolution.shooterRight.hoodAngle);
      Robot.telemetry().log("ShotSolution/Right/Omega", currentSolution.shooterRight.flywheelOmega);
      Robot.telemetry().log("ShotSolution/Right/TOF", currentSolution.shooterRight.tof);
      Robot.telemetry()
          .log("ShotSolution/Right/Clamped", currentSolution.shooterRight.hoodAngleClamped);
      Robot.telemetry()
          .log(
              "ShotSolution/Right/ArrivalConstraint",
              currentSolution.shooterRight.arrivalAngleConstrained);
    }
  }

  /** Ideal field-frame shooting direction and converged TOF for one shooter. */
  private record SolverResult(double phi, double tof) {}

  /**
   * Iteratively solve for the ideal field-frame shooting direction (φ) and converged time-of-flight
   * for a shooter whose bottom flywheel center is at {@code pivotPos}.
   */
  private static SolverResult solveDirection(Translation3d pivotPos, ChassisSpeeds fieldSpeeds) {
    final double vRx = fieldSpeeds.vxMetersPerSecond, vRy = fieldSpeeds.vyMetersPerSecond;
    double rawDx = m_targetCenter.getX() - pivotPos.getX();
    double rawDy = m_targetCenter.getY() - pivotPos.getY();
    double tof = Math.max(Math.hypot(rawDx, rawDy) / 8.0, 0.05);
    double phi = Math.atan2(rawDy, rawDx);
    double hood = (kMinHoodAngleRad + kMaxHoodAngleRad) / 2.0;

    for (int i = 0; i < kMaxIter; i++) {
      Translation3d ep = exitPoint(pivotPos, hood, phi);
      double effDx = (m_targetCenter.getX() - ep.getX()) - vRx * tof;
      double effDy = (m_targetCenter.getY() - ep.getY()) - vRy * tof;
      double dz = m_targetCenter.getZ() - ep.getZ();
      double D = Math.hypot(effDx, effDy);
      if (D < 1e-4) {
        phi = Math.atan2(rawDy, rawDx);
        tof = 0.05;
        break;
      }

      phi = Math.atan2(effDy, effDx);
      hood =
          clamp(
              Math.PI / 2.0 - (Math.atan2(dz + Math.sqrt(dz * dz + D * D), D)),
              kMinHoodAngleRad,
              kMaxHoodAngleRad);
      double la = Math.PI / 2.0 - hood;
      double v = minExitSpeed(D, dz, la);
      if (!Double.isFinite(v) || v <= 0) {
        hood = (hood <= kMinHoodAngleRad) ? kMaxHoodAngleRad : kMinHoodAngleRad;
        la = Math.PI / 2.0 - hood;
        v = minExitSpeed(D, dz, la);
      }
      double cl = Math.cos(la);
      double newT = (Double.isFinite(v) && v > 0 && cl > 1e-9) ? D / (v * cl) : tof;
      if (!Double.isFinite(newT) || newT <= 0) newT = tof;
      if (Math.abs(newT - tof) < kTofEpsilonSeconds) {
        tof = newT;
        break;
      }
      tof = newT;
    }
    return new SolverResult(phi, tof);
  }

  /**
   * @param hoodAngle Final mechanical hood angle (rad).
   * @param exitSpeed Ball exit speed (m/s).
   * @param hoodClamped Mechanical range was the active constraint.
   * @param arrivalConstrained Arrival angle was the active constraint.
   * @param effectiveMaxHood Upper hood bound after applying arrival constraint (rad). Passed to
   *     tolerance computation to cap the flatter direction.
   */
  private record PhysicsResult(
      double hoodAngle,
      double exitSpeed,
      boolean hoodClamped,
      boolean arrivalConstrained,
      double effectiveMaxHood,
      double tof) {}

  private static PhysicsResult solvePhysicsAlongDirection(
      Translation3d pivot, ChassisSpeeds fs, double phi, double tofGuess) {

    final double vRx = fs.vxMetersPerSecond, vRy = fs.vyMetersPerSecond;
    final double cP = Math.cos(phi), sP = Math.sin(phi);

    double tof = Math.max(tofGuess, 0.05);
    double hood = (kMinHoodAngleRad + kMaxHoodAngleRad) / 2.0;
    double speed = 0.0;
    boolean hCl = false, arrCl = false;
    double effMax = kMaxHoodAngleRad;

    for (int i = 0; i < kMaxIter; i++) {
      Translation3d ep = exitPoint(pivot, hood, phi);
      double effDx = (m_targetCenter.getX() - ep.getX()) - vRx * tof;
      double effDy = (m_targetCenter.getY() - ep.getY()) - vRy * tof;
      double dz = m_targetCenter.getZ() - ep.getZ();
      double D = effDx * cP + effDy * sP;

      if (D < 1e-4) {
        hood = kMaxHoodAngleRad;
        hCl = true;
        double la = Math.PI / 2.0 - hood;
        speed = minExitSpeed(1e-4, dz, la);
        tof = 1e-4 / (speed * Math.cos(la));
        effMax = hood;
        break;
      }

      // Arrival angle constraint
      double dNear = edgeDistance(-cP, -sP);
      effMax = kMaxHoodAngleRad;

      // Geometric near-edge constraint
      if (dNear > 1e-4) {
        double laMinGeo = Math.atan(kBallRadiusMeters / dNear + 2.0 * dz / D);
        effMax = Math.min(effMax, Math.PI / 2.0 - laMinGeo);
      }

      // Preferred minimum arrival angle
      if (m_preferredMinArrivalAngleRad > 1e-6) {
        double laMinPref = Math.atan(Math.tan(m_preferredMinArrivalAngleRad) + 2.0 * dz / D);
        effMax = Math.min(effMax, Math.PI / 2.0 - laMinPref);
      }

      if (effMax < kMinHoodAngleRad) {
        effMax = kMinHoodAngleRad;
        hCl = true;
        arrCl = true;
      }

      double optHood = Math.PI / 2.0 - (Math.atan2(dz + Math.sqrt(dz * dz + D * D), D));
      arrCl = optHood > effMax;
      hCl = (optHood < kMinHoodAngleRad) || arrCl;
      hood = clamp(optHood, kMinHoodAngleRad, effMax);

      double la = Math.PI / 2.0 - hood;
      speed = minExitSpeed(D, dz, la);
      if (!Double.isFinite(speed) || speed <= 0) {
        hood = (hood <= kMinHoodAngleRad) ? effMax : kMinHoodAngleRad;
        hCl = true;
        la = Math.PI / 2.0 - hood;
        speed = minExitSpeed(D, dz, la);
      }

      double cl = Math.cos(la);
      double newT = (Double.isFinite(speed) && speed > 0 && cl > 1e-9) ? D / (speed * cl) : tof;
      if (!Double.isFinite(newT) || newT <= 0) newT = tof;
      if (Math.abs(newT - tof) < kTofEpsilonSeconds) {
        tof = newT;
        break;
      }
      tof = newT;
    }
    return new PhysicsResult(hood, speed, hCl, arrCl, effMax, tof);
  }

  /**
   * Distance from the target centroid to the hexagon boundary in the given unit-vector direction
   * {@code (rdx, rdy)} (field XY plane, meters).
   *
   * @return distance in meters, or {@link Double#MAX_VALUE} if no intersection found.
   */
  private static double edgeDistance(double rdx, double rdy) {
    final double cx = m_targetCenter.getX(), cy = m_targetCenter.getY();
    double minT = Double.MAX_VALUE;
    int n = m_targetVertices.length;

    for (int i = 0; i < n; i++) {
      double ax = m_targetVertices[i].getX() - cx;
      double ay = m_targetVertices[i].getY() - cy;
      double bx = m_targetVertices[(i + 1) % n].getX() - cx;
      double by = m_targetVertices[(i + 1) % n].getY() - cy;
      double ex = bx - ax, ey = by - ay;

      // Solve: t·[rdx,rdy] = [ax,ay] + s·[ex,ey]
      // det = rdx·(−ey) − (−ex)·rdy = −rdx·ey + ex·rdy
      double det = -rdx * ey + ex * rdy;
      if (Math.abs(det) < 1e-10) continue;

      double t = (-ax * ey + ex * ay) / det;
      double s = (rdx * ay - ax * rdy) / det;

      if (t > 1e-9 && s >= -1e-9 && s <= 1.0 + 1e-9) {
        minT = Math.min(minT, t);
      }
    }
    return minT;
  }

  /**
   * Computes the ball exit point in field frame.
   *
   * @param pivotPos Bottom flywheel center in field frame.
   * @param hoodAngle Mechanical hood angle (radians above horizontal).
   * @param phi Field-frame shooting direction (yaw, radians).
   */
  private static Translation3d exitPoint(Translation3d pivotPos, double hoodAngle, double phi) {
    return new Translation3d(
        pivotPos.getX() + kExitOffsetMeters * Math.cos(hoodAngle) * Math.cos(phi),
        pivotPos.getY() + kExitOffsetMeters * Math.cos(hoodAngle) * Math.sin(phi),
        pivotPos.getZ() + kExitOffsetMeters * Math.sin(hoodAngle));
  }

  /**
   * Minimum exit speed (m/s) for horizontal distance D, vertical rise dz, and launch angle theta
   * (radians).
   *
   * @return m/s, or {@link Double#NaN} if unreachable at this angle.
   */
  private static double minExitSpeed(double D, double dz, double theta) {
    double den = 2.0 * Math.cos(theta) * Math.cos(theta) * (D * Math.tan(theta) - dz);
    return (den > 0.0) ? Math.sqrt(kG * D * D / den) : Double.NaN;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private ShooterMath2() {}
}
