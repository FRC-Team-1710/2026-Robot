package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.MathUtils;

public final class ShooterMath2 {

  /** Ball diameter: 5.91 in → meters. */
  private static final double BALL_DIAMETER_M = 5.91 * 0.0254;

  /** Ball radius: 2.955 in → meters. */
  private static final double BALL_RADIUS_M = BALL_DIAMETER_M / 2.0;

  /** Compression: 1 in → meters. */
  private static final double COMPRESSION_M = 1.00 * 0.0254;

  /** Bottom flywheel radius: 1.5 in → meters. */
  private static final double BOTTOM_WHEEL_RADIUS_M = 1.50 * 0.0254;

  /** Top flywheel radius: 1.0 in → meters. */
  private static final double TOP_WHEEL_RADIUS_M = 1.00 * 0.0254;

  /**
   * Distance from the bottom flywheel center to the ball center at the exit point (meters). The
   * ball center lies along the center-to-center line at:
   */
  private static final double EXIT_OFFSET_M =
      BOTTOM_WHEEL_RADIUS_M + (BALL_DIAMETER_M - COMPRESSION_M) / 2.0;

  /** Ball exit speed per unit flywheel. */
  public static final double SPEED_PER_OMEGA = (BOTTOM_WHEEL_RADIUS_M + TOP_WHEEL_RADIUS_M) / 2.0;

  /** Minimum mechanical hood angle (radians) — steepest shot. */
  private static final double MIN_HOOD_ANGLE_RAD = Math.toRadians(ShooterConstants.HOOD_MIN);

  /** Maximum mechanical hood angle (radians) — flattest shot. */
  private static final double MAX_HOOD_ANGLE_RAD = Math.toRadians(ShooterConstants.HOOD_MAX);

  /** Gravitational acceleration (m/s²). */
  private static final double G = 9.80665;

  /** Maximum number of iterations for the solver. */
  private static final int MAX_ITER = 20;

  /** Time-of-flight epsilon for the solver (seconds). */
  private static final double TOF_EPSILON_S = 1e-7;

  /** Finite-difference step for tolerance computation (radians / rad·s⁻¹). */
  private static final double FD_ANGLE = 1e-4;

  private static final double FD_OMEGA = 0.1;

  /**
   * Per-parameter shooting tolerances for a single shooter.
   *
   * @param headingToleranceLeft Robot can rotate CCW by this much.
   * @param headingToleranceRight Robot can rotate CW by this much.
   * @param hoodToleranceSteep Hood can decrease (go steeper) by this much.
   * @param hoodToleranceFlat Hood can increase (go flatter) by this much. Already bounded by the
   *     arrival-angle constraint.
   * @param flywheelToleranceFast Flywheel ω can increase by this much.
   * @param flywheelToleranceSlow Flywheel ω can decrease by this much.
   */
  public record ShooterTolerances(
      Angle headingToleranceLeft,
      Angle headingToleranceRight,
      Angle hoodToleranceSteep,
      Angle hoodToleranceFlat,
      AngularVelocity flywheelToleranceFast,
      AngularVelocity flywheelToleranceSlow) {}

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
      boolean arrivalAngleConstrained,
      ShooterTolerances tolerances) {
    /** Robot angle in tolerances */
    public boolean inTolerance(Rotation2d robotAngle) {
      return MathUtils.inRange(
          robotAngle.getRadians(),
          currentSolution.robotHeading.getRadians() - tolerances.hoodToleranceSteep.in(Radians),
          currentSolution.robotHeading.getRadians() + tolerances.hoodToleranceFlat.in(Radians));
    }

    /** Hood angle in tolerances */
    public boolean inTolerance(Angle hoodAngle) {
      return MathUtils.inRange(
          hoodAngle.in(Radians),
          hoodAngle.minus(tolerances.hoodToleranceSteep).in(Radians),
          hoodAngle.plus(tolerances.hoodToleranceFlat).in(Radians));
    }

    /** Flywheel omega in tolerances */
    public boolean inTolerance(AngularVelocity flywheelOmega) {
      return MathUtils.inRange(
          flywheelOmega.in(RadiansPerSecond),
          flywheelOmega.minus(tolerances.flywheelToleranceSlow).in(RadiansPerSecond),
          flywheelOmega.plus(tolerances.flywheelToleranceFast).in(RadiansPerSecond));
    }
  }

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
  private static Translation3d TARGET_CENTER = new Translation3d();

  /** Center of the hub */
  private static Translation3d[] TARGET_VERTICES = new Translation3d[0];

  /** The current solution for the dual shooter system. */
  public static DualShooterSolution currentSolution =
      new DualShooterSolution(
          Rotation2d.kZero,
          new ShooterSolution(
              Radians.of(0),
              RadiansPerSecond.of(0),
              Seconds.of(1),
              false,
              false,
              new ShooterTolerances(
                  Degrees.of(0),
                  Degrees.of(0),
                  Degrees.of(0),
                  Degrees.of(0),
                  DegreesPerSecond.of(0),
                  DegreesPerSecond.of(0))),
          new ShooterSolution(
              Radians.of(0),
              RadiansPerSecond.of(0),
              Seconds.of(1),
              false,
              false,
              new ShooterTolerances(
                  Degrees.of(0),
                  Degrees.of(0),
                  Degrees.of(0),
                  Degrees.of(0),
                  DegreesPerSecond.of(0),
                  DegreesPerSecond.of(0))));

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
    TARGET_VERTICES =
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
        Math.hypot(TARGET_CENTER.getX() - pivot1.getX(), TARGET_CENTER.getY() - pivot1.getY());
    double D2 =
        Math.hypot(TARGET_CENTER.getX() - pivot2.getX(), TARGET_CENTER.getY() - pivot2.getY());

    double phiComp =
        Math.atan2(
            (D1 * Math.sin(r1.phi) + D2 * Math.sin(r2.phi)) / (D1 + D2),
            (D1 * Math.cos(r1.phi) + D2 * Math.cos(r2.phi)) / (D1 + D2));

    double tofGuess = (r1.tof + r2.tof) / 2.0;

    PhysicsResult phys1 = solvePhysicsAlongDirection(pivot1, fieldSpeeds, phiComp, tofGuess);
    PhysicsResult phys2 = solvePhysicsAlongDirection(pivot2, fieldSpeeds, phiComp, tofGuess);

    double omega1 = phys1.exitSpeed / SPEED_PER_OMEGA;
    double omega2 = phys2.exitSpeed / SPEED_PER_OMEGA;

    currentSolution =
        new DualShooterSolution(
            new Rotation2d(phiComp),
            new ShooterSolution(
                Radians.of(phys1.hoodAngle),
                RadiansPerSecond.of(omega1),
                Seconds.of(phys1.tof),
                phys1.hoodClamped,
                phys1.arrivalConstrained,
                computeTolerances(
                    pivot1, phiComp, phys1.hoodAngle, omega1, phys1.effectiveMaxHood)),
            new ShooterSolution(
                Radians.of(phys2.hoodAngle),
                RadiansPerSecond.of(omega2),
                Seconds.of(phys2.tof),
                phys2.hoodClamped,
                phys2.arrivalConstrained,
                computeTolerances(
                    pivot2, phiComp, phys2.hoodAngle, omega2, phys2.effectiveMaxHood)));

    Robot.telemetry().log("ShotSolution/Heading", currentSolution.robotHeading, Rotation2d.struct);

    Robot.telemetry().log("ShotSolution/Left/Angle", currentSolution.shooterLeft.hoodAngle);
    Robot.telemetry().log("ShotSolution/Left/Omega", currentSolution.shooterLeft.flywheelOmega);
    Robot.telemetry().log("ShotSolution/Left/TOF", currentSolution.shooterLeft.tof);
    Robot.telemetry()
        .log("ShotSolution/Left/Clamped", currentSolution.shooterLeft.hoodAngleClamped);
    Robot.telemetry()
        .log(
            "ShotSolution/Left/ArrivalConstraint",
            currentSolution.shooterLeft.arrivalAngleConstrained);
    Robot.telemetry()
        .log(
            "ShotSolution/Left/Tolerances/HeadingToleranceLeft",
            currentSolution.shooterLeft.tolerances.headingToleranceLeft);
    Robot.telemetry()
        .log(
            "ShotSolution/Left/Tolerances/HeadingToleranceRight",
            currentSolution.shooterLeft.tolerances.headingToleranceRight);
    Robot.telemetry()
        .log(
            "ShotSolution/Left/Tolerances/HoodToleranceSteep",
            currentSolution.shooterLeft.tolerances.hoodToleranceSteep);
    Robot.telemetry()
        .log(
            "ShotSolution/Left/Tolerances/HoodToleranceFlat",
            currentSolution.shooterLeft.tolerances.hoodToleranceFlat);
    Robot.telemetry()
        .log(
            "ShotSolution/Left/Tolerances/FlywheelToleranceFast",
            currentSolution.shooterLeft.tolerances.flywheelToleranceFast);
    Robot.telemetry()
        .log(
            "ShotSolution/Left/Tolerances/FlywheelToleranceSlow",
            currentSolution.shooterLeft.tolerances.flywheelToleranceSlow);

    Robot.telemetry().log("ShotSolution/Right/Angle", currentSolution.shooterRight.hoodAngle);
    Robot.telemetry().log("ShotSolution/Right/Omega", currentSolution.shooterRight.flywheelOmega);
    Robot.telemetry().log("ShotSolution/Right/TOF", currentSolution.shooterRight.tof);
    Robot.telemetry()
        .log("ShotSolution/Right/Clamped", currentSolution.shooterRight.hoodAngleClamped);
    Robot.telemetry()
        .log(
            "ShotSolution/Right/ArrivalConstraint",
            currentSolution.shooterRight.arrivalAngleConstrained);
    Robot.telemetry()
        .log(
            "ShotSolution/Right/Tolerances/HeadingToleranceLeft",
            currentSolution.shooterRight.tolerances.headingToleranceLeft);
    Robot.telemetry()
        .log(
            "ShotSolution/Right/Tolerances/HeadingToleranceRight",
            currentSolution.shooterRight.tolerances.headingToleranceRight);
    Robot.telemetry()
        .log(
            "ShotSolution/Right/Tolerances/HoodToleranceSteep",
            currentSolution.shooterRight.tolerances.hoodToleranceSteep);
    Robot.telemetry()
        .log(
            "ShotSolution/Right/Tolerances/HoodToleranceFlat",
            currentSolution.shooterRight.tolerances.hoodToleranceFlat);
    Robot.telemetry()
        .log(
            "ShotSolution/Right/Tolerances/FlywheelToleranceFast",
            currentSolution.shooterRight.tolerances.flywheelToleranceFast);
    Robot.telemetry()
        .log(
            "ShotSolution/Right/Tolerances/FlywheelToleranceSlow",
            currentSolution.shooterRight.tolerances.flywheelToleranceSlow);
  }

  /** Ideal field-frame shooting direction and converged TOF for one shooter. */
  private record SolverResult(double phi, double tof) {}

  /**
   * Iteratively solve for the ideal field-frame shooting direction (φ) and converged time-of-flight
   * for a shooter whose bottom flywheel center is at {@code pivotPos}.
   */
  private static SolverResult solveDirection(Translation3d pivotPos, ChassisSpeeds fieldSpeeds) {
    final double vRx = fieldSpeeds.vxMetersPerSecond, vRy = fieldSpeeds.vyMetersPerSecond;
    double rawDx = TARGET_CENTER.getX() - pivotPos.getX();
    double rawDy = TARGET_CENTER.getY() - pivotPos.getY();
    double tof = Math.max(Math.hypot(rawDx, rawDy) / 8.0, 0.05);
    double phi = Math.atan2(rawDy, rawDx);
    double hood = (MIN_HOOD_ANGLE_RAD + MAX_HOOD_ANGLE_RAD) / 2.0;

    for (int i = 0; i < MAX_ITER; i++) {
      Translation3d ep = exitPoint(pivotPos, hood, phi);
      double effDx = (TARGET_CENTER.getX() - ep.getX()) - vRx * tof;
      double effDy = (TARGET_CENTER.getY() - ep.getY()) - vRy * tof;
      double dz = TARGET_CENTER.getZ() - ep.getZ();
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
              MIN_HOOD_ANGLE_RAD,
              MAX_HOOD_ANGLE_RAD);
      double la = Math.PI / 2.0 - hood;
      double v = minExitSpeed(D, dz, la);
      if (!Double.isFinite(v) || v <= 0) {
        hood = (hood <= MIN_HOOD_ANGLE_RAD) ? MAX_HOOD_ANGLE_RAD : MIN_HOOD_ANGLE_RAD;
        la = Math.PI / 2.0 - hood;
        v = minExitSpeed(D, dz, la);
      }
      double cl = Math.cos(la);
      double newT = (Double.isFinite(v) && v > 0 && cl > 1e-9) ? D / (v * cl) : tof;
      if (!Double.isFinite(newT) || newT <= 0) newT = tof;
      if (Math.abs(newT - tof) < TOF_EPSILON_S) {
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
    double hood = (MIN_HOOD_ANGLE_RAD + MAX_HOOD_ANGLE_RAD) / 2.0;
    double speed = 0.0;
    boolean hCl = false, arrCl = false;
    double effMax = MAX_HOOD_ANGLE_RAD;

    for (int i = 0; i < MAX_ITER; i++) {
      Translation3d ep = exitPoint(pivot, hood, phi);
      double effDx = (TARGET_CENTER.getX() - ep.getX()) - vRx * tof;
      double effDy = (TARGET_CENTER.getY() - ep.getY()) - vRy * tof;
      double dz = TARGET_CENTER.getZ() - ep.getZ();
      double D = effDx * cP + effDy * sP;

      if (D < 1e-4) {
        hood = MAX_HOOD_ANGLE_RAD;
        hCl = true;
        double la = Math.PI / 2.0 - hood;
        speed = minExitSpeed(1e-4, dz, la);
        tof = 1e-4 / (speed * Math.cos(la));
        effMax = hood;
        break;
      }

      // Arrival angle constraint
      double dNear = edgeDistance(-cP, -sP);
      effMax = MAX_HOOD_ANGLE_RAD;
      if (dNear > 1e-4) {
        double laMin = Math.atan(BALL_RADIUS_M / dNear + 2.0 * dz / D);
        effMax = Math.min(MAX_HOOD_ANGLE_RAD, Math.PI / 2.0 - laMin);
        if (effMax < MIN_HOOD_ANGLE_RAD) {
          effMax = MIN_HOOD_ANGLE_RAD;
          hCl = true;
          arrCl = true;
        }
      }

      double optHood = Math.PI / 2.0 - (Math.atan2(dz + Math.sqrt(dz * dz + D * D), D));
      arrCl = optHood > effMax;
      hCl = (optHood < MIN_HOOD_ANGLE_RAD) || arrCl;
      hood = clamp(optHood, MIN_HOOD_ANGLE_RAD, effMax);

      double la = Math.PI / 2.0 - hood;
      speed = minExitSpeed(D, dz, la);
      if (!Double.isFinite(speed) || speed <= 0) {
        hood = (hood <= MIN_HOOD_ANGLE_RAD) ? effMax : MIN_HOOD_ANGLE_RAD;
        hCl = true;
        la = Math.PI / 2.0 - hood;
        speed = minExitSpeed(D, dz, la);
      }

      double cl = Math.cos(la);
      double newT = (Double.isFinite(speed) && speed > 0 && cl > 1e-9) ? D / (speed * cl) : tof;
      if (!Double.isFinite(newT) || newT <= 0) newT = tof;
      if (Math.abs(newT - tof) < TOF_EPSILON_S) {
        tof = newT;
        break;
      }
      tof = newT;
    }
    return new PhysicsResult(hood, speed, hCl, arrCl, effMax, tof);
  }

  /**
   * Computes per-parameter tolerances via finite differences around the nominal solution point.
   *
   * @param pivot Bottom flywheel center in field frame.
   * @param phi Field-frame shooting direction (rad).
   * @param hoodAngle Nominal mechanical hood angle (rad).
   * @param omega Nominal flywheel ω (rad/s).
   * @param effectiveMaxHood Upper hood bound after arrival constraint (rad).
   */
  private static ShooterTolerances computeTolerances(
      Translation3d pivot, double phi, double hoodAngle, double omega, double effectiveMaxHood) {

    final double v = omega * SPEED_PER_OMEGA;
    final double theta = Math.PI / 2.0 - hoodAngle;
    final double cP = Math.cos(phi), sP = Math.sin(phi);

    Translation3d exitNom = exitPoint(pivot, hoodAngle, phi);
    double[] nom = landingOffset(exitNom, v, theta, phi);

    // If the nominal trajectory can't reach the target, return zero tolerances.
    if (nom == null)
      return new ShooterTolerances(
          Degrees.of(0),
          Degrees.of(0),
          Degrees.of(0),
          Degrees.of(0),
          DegreesPerSecond.of(0),
          DegreesPerSecond.of(0));

    // ------------------------------------------------------------------
    // Shrunk-hexagon clearances:
    //   c = (distance from centroid to hexagon edge in direction d) − ball_radius
    // The ball center must stay within this shrunk hexagon.
    // ------------------------------------------------------------------
    double cFar = Math.max(0, edgeDistance(cP, sP) - BALL_RADIUS_M); // forward (away from shooter)
    double cNear = Math.max(0, edgeDistance(-cP, -sP) - BALL_RADIUS_M); // backward (toward shooter)
    double cLeft = Math.max(0, edgeDistance(-sP, cP) - BALL_RADIUS_M); // CCW side
    double cRight = Math.max(0, edgeDistance(sP, -cP) - BALL_RADIUS_M); // CW side

    // ------------------------------------------------------------------
    // HEADING TOLERANCE
    //
    // Rotating CCW (φ → φ+dφ) displaces the landing point to the left.
    // Rotating CW  (φ → φ−dφ) displaces the landing point to the right.
    //
    // Lateral sensitivity (m per rad):
    //   left  = dot(off − nom, left_dir)  / dφ   where left_dir  = (−sP,  cP)
    //   right = dot(off − nom, right_dir) / dφ   where right_dir = ( sP, −cP)
    // ------------------------------------------------------------------
    double[] offPhiL =
        landingOffset(exitPoint(pivot, hoodAngle, phi + FD_ANGLE), v, theta, phi + FD_ANGLE);
    double[] offPhiR =
        landingOffset(exitPoint(pivot, hoodAngle, phi - FD_ANGLE), v, theta, phi - FD_ANGLE);

    // Left sensitivity: positive when ball goes left as phi increases
    double sensL =
        offPhiL == null
            ? 1.0
            : (-(offPhiL[0] - nom[0]) * sP + (offPhiL[1] - nom[1]) * cP) / FD_ANGLE;
    // Right sensitivity: positive when ball goes right as phi decreases
    double sensR =
        offPhiR == null
            ? 1.0
            : ((offPhiR[0] - nom[0]) * sP - (offPhiR[1] - nom[1]) * cP) / FD_ANGLE;

    double headingTolLeft = cLeft / Math.max(sensL, 1e-9);
    double headingTolRight = cRight / Math.max(sensR, 1e-9);

    // ------------------------------------------------------------------
    // HOOD TOLERANCE
    //
    // Steeper (hood decreases by dA): exit point shifts, θ increases.
    // Flatter (hood increases by dA): exit point shifts, θ decreases.
    //
    // Along-barrel sensitivity (m per rad of hood change):
    //   steep_sens = dot(off − nom, fwd_dir) / dA
    //   flat_sens  = dot(off − nom, fwd_dir) / dA
    //
    // Positive → ball lands further (limited by cFar).
    // Negative → ball lands shorter (limited by cNear).
    //
    // Flywheel speed is held constant; only the hood moves.
    // ------------------------------------------------------------------
    double steepHood = Math.max(MIN_HOOD_ANGLE_RAD, hoodAngle - FD_ANGLE);
    double flatHood = Math.min(effectiveMaxHood, hoodAngle + FD_ANGLE);

    double[] offSteep =
        landingOffset(exitPoint(pivot, steepHood, phi), v, Math.PI / 2.0 - steepHood, phi);
    double[] offFlat =
        landingOffset(exitPoint(pivot, flatHood, phi), v, Math.PI / 2.0 - flatHood, phi);

    double sensSteep =
        offSteep == null
            ? 0.0
            : ((offSteep[0] - nom[0]) * cP + (offSteep[1] - nom[1]) * sP) / FD_ANGLE;
    double sensFlat =
        offFlat == null
            ? 0.0
            : ((offFlat[0] - nom[0]) * cP + (offFlat[1] - nom[1]) * sP) / FD_ANGLE;

    double hoodTolSteep = alongTolerance(sensSteep, cFar, cNear);
    double hoodTolFlat = alongTolerance(sensFlat, cFar, cNear);

    // Clamp to mechanical range and arrival-angle constraint
    hoodTolSteep = clamp(hoodTolSteep, 0, hoodAngle - MIN_HOOD_ANGLE_RAD);
    hoodTolFlat = clamp(hoodTolFlat, 0, effectiveMaxHood - hoodAngle);

    // ------------------------------------------------------------------
    // FLYWHEEL TOLERANCE
    //
    // Faster (ω → ω+dω): ball travels further.
    // Slower (ω → ω−dω): ball travels shorter.
    //
    // Hood is held constant; only exit speed changes.
    // ------------------------------------------------------------------
    double dV = FD_OMEGA * SPEED_PER_OMEGA;

    double[] offFast = landingOffset(exitNom, v + dV, theta, phi);
    double[] offSlow = landingOffset(exitNom, v - dV, theta, phi);

    double sensFast =
        offFast == null
            ? 0.0
            : ((offFast[0] - nom[0]) * cP + (offFast[1] - nom[1]) * sP) / FD_OMEGA;
    double sensSlow =
        offSlow == null
            ? 0.0
            : ((offSlow[0] - nom[0]) * cP + (offSlow[1] - nom[1]) * sP)
                / (-FD_OMEGA); // per rad/s of reduction

    double flywheelTolFast = Math.max(0, alongTolerance(sensFast, cFar, cNear));
    double flywheelTolSlow = Math.max(0, alongTolerance(sensSlow, cFar, cNear));

    return new ShooterTolerances(
        Radians.of(headingTolLeft),
        Radians.of(headingTolRight),
        Radians.of(hoodTolSteep),
        Radians.of(hoodTolFlat),
        RadiansPerSecond.of(flywheelTolFast),
        RadiansPerSecond.of(flywheelTolSlow));
  }

  /**
   * Given a forward/backward sensitivity (m per unit of parameter change, positive = toward the far
   * edge, negative = toward the near edge) and the two clearances, returns the tolerance for that
   * direction.
   */
  private static double alongTolerance(double sens, double cFar, double cNear) {
    if (Math.abs(sens) < 1e-9) return Double.MAX_VALUE; // effectively insensitive
    return (sens >= 0) ? cFar / sens : cNear / (-sens);
  }

  /**
   * Computes the ball's landing offset [dx, dy] from the target centroid (in field frame) for a
   * given exit condition, at Z = target height.
   *
   * @return {@code [dx, dy]} in meters, or {@code null} if the target height is unreachable with
   *     the given exit speed and angle.
   */
  private static double[] landingOffset(
      Translation3d exitPos, double v, double launchAngle, double phi) {

    double vz = v * Math.sin(launchAngle);
    double disc = vz * vz - 2.0 * G * (TARGET_CENTER.getZ() - exitPos.getZ());
    if (disc < 0) return null;

    double T = (vz + Math.sqrt(disc)) / G;
    if (T <= 0) {
      T = (vz - Math.sqrt(disc)) / G;
      if (T <= 0) return null;
    }

    return new double[] {
      exitPos.getX() + v * Math.cos(launchAngle) * Math.cos(phi) * T - TARGET_CENTER.getX(),
      exitPos.getY() + v * Math.cos(launchAngle) * Math.sin(phi) * T - TARGET_CENTER.getY()
    };
  }

  /**
   * Distance from the target centroid to the hexagon boundary in the given unit-vector direction
   * {@code (rdx, rdy)} (field XY plane, meters).
   *
   * @return distance in meters, or {@link Double#MAX_VALUE} if no intersection found.
   */
  private static double edgeDistance(double rdx, double rdy) {
    final double cx = TARGET_CENTER.getX(), cy = TARGET_CENTER.getY();
    double minT = Double.MAX_VALUE;
    int n = TARGET_VERTICES.length;

    for (int i = 0; i < n; i++) {
      double ax = TARGET_VERTICES[i].getX() - cx;
      double ay = TARGET_VERTICES[i].getY() - cy;
      double bx = TARGET_VERTICES[(i + 1) % n].getX() - cx;
      double by = TARGET_VERTICES[(i + 1) % n].getY() - cy;
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
        pivotPos.getX() + EXIT_OFFSET_M * Math.cos(hoodAngle) * Math.cos(phi),
        pivotPos.getY() + EXIT_OFFSET_M * Math.cos(hoodAngle) * Math.sin(phi),
        pivotPos.getZ() + EXIT_OFFSET_M * Math.sin(hoodAngle));
  }

  /**
   * Minimum exit speed (m/s) for horizontal distance D, vertical rise dz, and launch angle theta
   * (radians).
   *
   * @return m/s, or {@link Double#NaN} if unreachable at this angle.
   */
  private static double minExitSpeed(double D, double dz, double theta) {
    double den = 2.0 * Math.cos(theta) * Math.cos(theta) * (D * Math.tan(theta) - dz);
    return (den > 0.0) ? Math.sqrt(G * D * D / den) : Double.NaN;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  private ShooterMath2() {}
}
