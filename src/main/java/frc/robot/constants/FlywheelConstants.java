// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Settings for the Flywheel (shooter wheels).
 *
 * <p>Controls how fast the shooter spins:
 * <ul>
 *   <li>Shooting speeds for different distances
 *   <li>How close the speed needs to be to "at target"
 *   <li>Control values for speed control
 * </ul>
 */
public final class FlywheelConstants {

  // ==================== Shooting Speeds ====================

  /** How fast to spin for shooting (rotations per second) */
  public static final double SHOOTING_SPEED_RPS = 25.0;

  /** Slow speed for amp scoring (rotations per second) */
  public static final double AMP_SPEED_RPS = 5.0;

  /** Fast speed for far field shooting (rotations per second) */
  public static final double FAR_SHOOTING_SPEED_RPS = 35.0;

  // ==================== Tolerances ====================

  /** How close the speed needs to be to count as "at target" (RPS) */
  public static final double VELOCITY_TOLERANCE_RPS = 0.25;

  // ==================== PID Control Values ====================

  /** Static friction compensation */
  public static final double kS = 0.0;

  /** Velocity feedforward (predicts voltage needed for a speed) */
  public static final double kV = 0.125;

  /** Proportional gain (corrects speed errors) */
  public static final double kP = 0.0;

  // ==================== Motion Magic (Speed Limits) ====================

  /** Maximum speed the flywheel can reach (rotations per second) */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0;

  /** How fast the flywheel can speed up (rotations per second²) */
  public static final double MOTION_MAGIC_ACCELERATION = 1000.0;

  private FlywheelConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
