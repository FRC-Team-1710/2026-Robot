// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Constants for autonomous routines and path following.
 *
 * <p>Centralizes all autonomous-related configuration including:
 * <ul>
 *   <li>Robot physical properties (mass, MOI)
 *   <li>Motion constraints (velocity, acceleration)
 *   <li>Path following parameters
 * </ul>
 */
public final class AutoConstants {

  // ==================== Robot Physical Properties ====================

  /** Robot mass in pounds (used for wheel force calculations) */
  public static final double ROBOT_MASS_LBS = 125.0;

  /** Robot moment of inertia in kg·m² (used for rotational dynamics). If you don't know leave at 6*/
  public static final double MOMENT_OF_INERTIA_KG_M2 = 6.0;

  // ==================== Linear Motion Constraints ====================

  /** Maximum linear velocity in meters per second */
  public static final double MAX_LINEAR_VELOCITY_MPS = 3.0;

  /** Maximum linear acceleration in meters per second² */
  public static final double MAX_LINEAR_ACCELERATION_MPS2 = 3.0;

  // ==================== Angular Motion Constraints ====================

  /** Maximum angular velocity in radians per second (π rad/s = 0.5 rotations/sec) */
  public static final double MAX_ANGULAR_VELOCITY_RAD_S = Math.PI;

  /** Maximum angular acceleration in radians per second² (2π rad/s² = 1 rotation/sec²) */
  public static final double MAX_ANGULAR_ACCELERATION_RAD_S2 = 2 * Math.PI;

  // ==================== Path Following PID Gains ====================
  // Used by LinearPathRequest for feedback corrections

  /** X position PID proportional gain */
  public static final double X_CONTROLLER_KP = 10.0;

  /** Y position PID proportional gain */
  public static final double Y_CONTROLLER_KP = 10.0;

  /** Theta (rotation) PID proportional gain */
  public static final double THETA_CONTROLLER_KP = 7.0;

  private AutoConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
