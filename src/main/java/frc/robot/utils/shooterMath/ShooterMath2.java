package frc.robot.utils.shooterMath;

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

/**
 * Calculates optimal shooter parameters (robot heading, hood angles, flywheel ω values) to score
 * into a hexagonal target from any field position, supporting either a single shooter or two
 * fixed-direction shooters sharing a common drive-base heading.
 *
 * <p><b>Mechanism</b>
 *
 * <ul>
 *   <li>Two mechanically-linked flywheels (1:1). The top flywheel pivots around the bottom flywheel
 *       center at a fixed center-to-center distance:
 *       <pre>  pivot_dist = r_bottom + r_top + (ball_diameter − compression)</pre>
 *       The ball exits <em>perpendicular</em> to the center-to-center line:
 *       <pre>  θ_launch = π/2 − α_hood</pre>
 *       A <em>smaller</em> hood angle → steeper launch; a <em>larger</em> angle → flatter.
 *   <li>Ball exit speed (no-slip, two-wheel):
 *       <pre>  v_ball = ω × (r_bottom + r_top) / 2</pre>
 *   <li>Minimum-exit-speed launch angle (unconstrained optimum):
 *       <pre>  θ_opt = atan( (H + √(H² + D²)) / D )</pre>
 *       Clamped to the mechanical hood range after conversion to hood angle.
 *   <li>Shoot-while-moving: the robot's field-frame velocity offsets the ball's horizontal
 *       velocity. The solver iterates on time-of-flight (TOF) until convergence (typically 3–5
 *       iterations).
 * </ul>
 *
 * <p><b>Dual-shooter heading compromise</b><br>
 * Both shooters face the same direction relative to the robot (no turrets), so only one robot
 * heading is possible at a time. The solver runs independently for each shooter to find each
 * shooter's ideal field-frame shooting direction (φ₁, φ₂), then selects the compromise heading that
 * minimises the sum of squared lateral misses at the target plane. For separations of a few inches
 * and target distances of several metres, this is indistinguishable from a simple angular average,
 * but the weighted form handles asymmetric cases (different distances, one shooter clamped)
 * correctly. After the heading is fixed, each shooter's physics is re-solved along the shared
 * shooting direction to obtain accurate per-shooter hood angles and flywheel speeds. </pre>
 */
public final class ShooterMath2 {

  // =========================================================================
  // FLYWHEEL & BALL CONSTANTS
  // =========================================================================

  /** Ball diameter: 5.91 in → meters. */
  public static final double BALL_DIAMETER_M = 5.91 * 0.0254;

  /** Compression: 1 in → meters. */
  public static final double COMPRESSION_M = 1.00 * 0.0254;

  /** Bottom flywheel radius: 1.5 in → meters. */
  public static final double BOTTOM_WHEEL_RADIUS_M = 1.50 * 0.0254;

  /** Top flywheel radius: 1.0 in → meters. */
  public static final double TOP_WHEEL_RADIUS_M = 1.00 * 0.0254;

  /**
   * Ball exit speed per unit flywheel ω (m / (rad/s)).
   *
   * <pre>  v_ball = ω × (r_bottom + r_top) / 2  </pre>
   */
  private static final double SPEED_PER_OMEGA = (BOTTOM_WHEEL_RADIUS_M + TOP_WHEEL_RADIUS_M) / 2.0;

  // =========================================================================
  // HOOD GEOMETRY
  //
  //   pivot_dist = r_bottom + r_top + (ball_diameter − compression)
  //   θ_launch   = π/2 − α_hood
  //   α_hood     = π/2 − θ_launch
  //
  //   Smaller hood angle → steeper launch.
  //   Larger  hood angle → flatter  launch.
  // =========================================================================

  /**
   * Fixed center-to-center distance between flywheel centers (meters).
   *
   * <pre>
   *   = r_bottom + r_top + (ball_diameter − compression)
   *   ≈ 1.5 + 1.0 + (5.91 − 1.0)  [inches]  ≈ 7.41 in  ≈ 0.188 m
   * </pre>
   */
  public static final double PIVOT_DISTANCE_M =
      BOTTOM_WHEEL_RADIUS_M + TOP_WHEEL_RADIUS_M + (BALL_DIAMETER_M - COMPRESSION_M);

  /** Minimum mechanical hood angle (radians) — steepest shot. stop. */
  public static final double MIN_HOOD_ANGLE_RAD = Math.toRadians(27.5);

  /** Maximum mechanical hood angle (radians) — flattest shot. stop. */
  public static final double MAX_HOOD_ANGLE_RAD = Math.toRadians(41.0);

  // =========================================================================
  // PHYSICS & SOLVER
  // =========================================================================

  private static final double G = 9.80665; // m/s²
  private static final int MAX_ITER = 20;
  private static final double TOF_EPSILON_S = 1e-7;

  // =========================================================================
  // RESULT TYPES
  // =========================================================================

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
   * <p>The single {@link #robotHeading()} is the compromise heading that minimises the sum of
   * squared lateral misses for both shooters. Each shooter then has its own {@link ShooterSolution}
   * containing the hood angle and flywheel speed required to hit the target centre from its
   * specific position at that heading.
   *
   * @param robotHeading Field-frame heading to command to the drive base.
   * @param shooterLeft Solution for the first shooter (corresponds to {@code robotToShooter1}).
   * @param shooterRight Solution for the second shooter (corresponds to {@code robotToShooter2}).
   */
  public record DualShooterSolution(
      Rotation2d robotHeading, ShooterSolution shooterLeft, ShooterSolution shooterRight) {}

  // =========================================================================
  // GLOBAL VARIABLES FOR SOLVERS
  // =========================================================================

  /** Center of the hub */
  private static Translation3d TARGET_CENTER = new Translation3d();

  // =========================================================================
  // PUBLIC SOLUTIONS
  // =========================================================================

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

  // =========================================================================
  // PUBLIC API — DUAL SHOOTER
  // =========================================================================

  /**
   * Compute optimal shooter parameters for two fixed-direction shooters that share a single
   * drive-base heading.
   *
   * <p>Both {@code robotToShooter1} and {@code robotToShooter2} must have the same yaw component
   * (both shooters face the same direction relative to the robot). Only the translation (position
   * offset on the robot) should differ.
   *
   * <p><b>Heading selection</b><br>
   * Each shooter's unconstrained optimum defines an ideal field-frame shooting direction (φ₁, φ₂).
   * The compromise heading minimises the sum of squared lateral misses at the target, weighted by
   * each shooter's effective horizontal distance to the target (closer shooters are more sensitive
   * to angular error):
   *
   * <pre>
   *   φ_compromise = (D₁·φ₁ + D₂·φ₂) / (D₁ + D₂)
   * </pre>
   *
   * With separations of a few inches this is essentially a simple average, but the weighting
   * handles edge cases cleanly.
   *
   * @param robotPose Current robot 2-D pose.
   * @param robotToShooterLeft Transform for the left shooter.
   * @param robotToShooterRight Transform for the right shooter.
   * @param fieldSpeeds Chassis speeds in the <em>field</em> frame.
   * @return {@link DualShooterSolution} with one robot heading and per-shooter physics.
   */
  public static void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {

    currentPose = robotPose;
    currentSpeeds = fieldSpeeds;

    // ---- Hub center ----
    TARGET_CENTER =
        Alliance.redAlliance ? FieldConstants.kHubCenterRed : FieldConstants.kHubCenterBlue;

    // ---- Shooter positions in field frame ----
    Translation3d pos1 =
        new Pose3d(robotPose).transformBy(ShooterConstants.kLEFT_SHOOTER_OFFSET).getTranslation();
    Translation3d pos2 =
        new Pose3d(robotPose).transformBy(ShooterConstants.kRIGHT_SHOOTER_OFFSET).getTranslation();

    // ---- Per-shooter unconstrained direction solve ----
    SolverResult r1 = solveDirection(pos1, fieldSpeeds);
    SolverResult r2 = solveDirection(pos2, fieldSpeeds);

    // ---- Compromise heading ----
    // Weight each shooter's ideal direction by its effective horizontal distance
    // to the target (smaller angular error allowed for the closer shooter).
    double D1 = Math.hypot(TARGET_CENTER.getX() - pos1.getX(), TARGET_CENTER.getY() - pos1.getY());
    double D2 = Math.hypot(TARGET_CENTER.getX() - pos2.getX(), TARGET_CENTER.getY() - pos2.getY());

    // Weighted circular-mean of the two angles.
    // For the tiny angular differences expected here this is equivalent to a
    // weighted arithmetic mean, but the sin/cos form is numerically safe
    // across the ±π wrap.
    double totalW = D1 + D2;
    double phiComp =
        Math.atan2(
            (D1 * Math.sin(r1.phi) + D2 * Math.sin(r2.phi)) / totalW,
            (D1 * Math.cos(r1.phi) + D2 * Math.cos(r2.phi)) / totalW);

    // Both shooters share the same yaw offset (they face the same robot-relative
    // direction). Use shooter 1's value; assert equality during testing.
    Rotation2d robotHeading =
        new Rotation2d(
            phiComp
                - ShooterConstants.kLEFT_SHOOTER_OFFSET.getRotation().toRotation2d().getRadians());

    // ---- Per-shooter physics at the compromise heading ----
    // Use the average TOF from the two independent solves as the starting guess
    // for the re-solve (converges in 1–2 extra iterations).
    double tofGuess = (r1.tof + r2.tof) / 2.0;

    PhysicsResult phys1 = solvePhysicsAlongDirection(pos1, fieldSpeeds, phiComp, tofGuess);
    PhysicsResult phys2 = solvePhysicsAlongDirection(pos2, fieldSpeeds, phiComp, tofGuess);

    currentSolution =
        new DualShooterSolution(
            robotHeading,
            new ShooterSolution(
                robotHeading,
                Radians.of(phys1.hoodAngle),
                RadiansPerSecond.of(phys1.exitSpeed / SPEED_PER_OMEGA),
                phys1.clamped),
            new ShooterSolution(
                robotHeading,
                Radians.of(phys2.hoodAngle),
                RadiansPerSecond.of(phys2.exitSpeed / SPEED_PER_OMEGA),
                phys2.clamped));
  }

  // =========================================================================
  // SOLVER INTERNALS
  // =========================================================================

  /**
   * Intermediate result from the direction solver: the ideal field-frame shooting direction and the
   * converged time-of-flight.
   */
  private record SolverResult(double phi, double tof) {}

  /**
   * Iteratively solve for the ideal field-frame shooting direction (φ) and converged time-of-flight
   * for a shooter at {@code shooterPos}, accounting for the robot's field-frame velocity.
   *
   * <p>This is the unconstrained direction optimum. Hood angle clamping is handled separately in
   * {@link #solvePhysicsAlongDirection}.
   */
  private static SolverResult solveDirection(Translation3d shooterPos, ChassisSpeeds fieldSpeeds) {

    final double rawDx = TARGET_CENTER.getX() - shooterPos.getX();
    final double rawDy = TARGET_CENTER.getY() - shooterPos.getY();
    final double dz = TARGET_CENTER.getZ() - shooterPos.getZ();

    double tof = Math.max(Math.hypot(rawDx, rawDy) / 8.0, 0.05);
    double phi = Math.atan2(rawDy, rawDx);

    for (int iter = 0; iter < MAX_ITER; iter++) {
      double effDx = rawDx - fieldSpeeds.vxMetersPerSecond * tof;
      double effDy = rawDy - fieldSpeeds.vyMetersPerSecond * tof;
      double D = Math.hypot(effDx, effDy);

      if (D < 1e-4) {
        phi = Math.atan2(rawDy, rawDx);
        tof = 0.05;
        break;
      }

      phi = Math.atan2(effDy, effDx);

      // Optimal (unconstrained) launch angle for minimum exit speed;
      // Clamp to mechanical hood range
      double hoodAngle =
          clamp(
              Math.PI / 2.0 - Math.atan2(dz + Math.sqrt(dz * dz + D * D), D),
              MIN_HOOD_ANGLE_RAD,
              MAX_HOOD_ANGLE_RAD);
      double launchAngle = Math.PI / 2.0 - hoodAngle;

      double exitSpeed = minExitSpeed(D, dz, launchAngle);
      if (!Double.isFinite(exitSpeed) || exitSpeed <= 0.0) {
        // Try opposite hood limit
        hoodAngle = (hoodAngle <= MIN_HOOD_ANGLE_RAD) ? MAX_HOOD_ANGLE_RAD : MIN_HOOD_ANGLE_RAD;
        launchAngle = Math.PI / 2.0 - hoodAngle;
        exitSpeed = minExitSpeed(D, dz, launchAngle);
      }

      double cosL = Math.cos(launchAngle);
      double newTof =
          (Double.isFinite(exitSpeed) && exitSpeed > 0 && cosL > 1e-9)
              ? D / (exitSpeed * cosL)
              : tof;

      if (Math.abs(newTof - tof) < TOF_EPSILON_S) {
        tof = newTof;
        break;
      }
      tof = newTof;
    }

    return new SolverResult(phi, tof);
  }

  /** Intermediate result from the per-shooter physics solve. */
  private record PhysicsResult(double hoodAngle, double exitSpeed, boolean clamped) {}

  /**
   * Given a fixed field-frame shooting direction φ (the robot heading has already been decided),
   * solve for the hood angle and exit speed required for the ball from {@code shooterPos} to reach
   * {@link #TARGET_CENTER}.
   *
   * <p>The effective horizontal distance is the <em>projection</em> of the displacement vector onto
   * φ. The perpendicular component is the lateral miss (acceptable if within the target's opening
   * radius). The TOF iteration starts from {@code tofGuess} and converges in 1–3 extra iterations.
   *
   * @param shooterPos Shooter exit point in field frame.
   * @param fieldSpeeds Field-frame chassis speeds.
   * @param phi Fixed field-frame shooting direction (radians).
   * @param tofGuess Initial TOF estimate (seconds).
   * @return Hood angle, exit speed, and clamped flag.
   */
  private static PhysicsResult solvePhysicsAlongDirection(
      Translation3d shooterPos, ChassisSpeeds fieldSpeeds, double phi, double tofGuess) {

    final double dz = TARGET_CENTER.getZ() - shooterPos.getZ();

    double tof = Math.max(tofGuess, 0.05);
    double hood = MIN_HOOD_ANGLE_RAD;
    double speed = 0.0;
    boolean clamp = false;

    for (int iter = 0; iter < MAX_ITER; iter++) {
      // Effective displacement after subtracting robot motion.
      // Project onto the shooting direction to get the along-barrel distance.
      // The perpendicular component is lateral miss (we accept it; the target
      // is large and the two shooters are only inches apart).
      double D =
          (TARGET_CENTER.getX() - shooterPos.getX() - fieldSpeeds.vxMetersPerSecond * tof)
                  * Math.cos(phi)
              + (TARGET_CENTER.getY() - shooterPos.getY() - fieldSpeeds.vyMetersPerSecond * tof)
                  * Math.sin(phi);

      if (D < 1e-4) {
        hood = MAX_HOOD_ANGLE_RAD;
        clamp = true;
        double la = Math.PI / 2.0 - hood;
        speed = minExitSpeed(1e-4, dz, la);
        tof = 1e-4 / (speed * Math.cos(la));
        break;
      }

      // Optimal (unconstrained) launch angle
      // Convert to hood angle and clamp
      double optHood = Math.PI / 2.0 - Math.atan2(dz + Math.sqrt(dz * dz + D * D), D);
      clamp = (optHood < MIN_HOOD_ANGLE_RAD || optHood > MAX_HOOD_ANGLE_RAD);
      hood = clamp(optHood, MIN_HOOD_ANGLE_RAD, MAX_HOOD_ANGLE_RAD);

      double launch = Math.PI / 2.0 - hood;
      speed = minExitSpeed(D, dz, launch);

      if (!Double.isFinite(speed) || speed <= 0.0) {
        hood = (hood <= MIN_HOOD_ANGLE_RAD) ? MAX_HOOD_ANGLE_RAD : MIN_HOOD_ANGLE_RAD;
        clamp = true;
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

    return new PhysicsResult(hood, speed, clamp);
  }

  // =========================================================================
  // PRIVATE HELPERS
  // =========================================================================

  /**
   * Minimum exit speed for horizontal distance D, vertical rise dz, and launch angle theta.
   *
   * <pre>  v² = g·D² / (2·cos²θ·(D·tanθ − dz))  </pre>
   *
   * @return m/s, or {@link Double#NaN} if unreachable at this angle.
   */
  private static double minExitSpeed(double D, double dz, double theta) {
    double cosT = Math.cos(theta);
    double den = 2.0 * cosT * cosT * (D * Math.tan(theta) - dz);
    return (den > 0.0) ? Math.sqrt(G * D * D / den) : Double.NaN;
  }

  /** Your average joe clamper */
  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
