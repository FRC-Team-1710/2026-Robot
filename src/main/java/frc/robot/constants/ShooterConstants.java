// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ShooterConstants {
  // ==================== PID Control Values ====================

  /** Static friction compensation */
  public static final double kS = 4.0;

  /** Velocity feedforward (predicts voltage needed for a speed) */
  public static final double kV = 0.035;

  /** Proportional gain (corrects speed errors) */
  public static final double kP = 2.0;

  // ==================== Current Limits ====================
  public static final int FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60;
  public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 80;

  public static final int HOOD_SUPPLY_CURRENT_LIMIT = 20;
  public static final int HOOD_STATOR_CURRENT_LIMIT = 20;

  // ==================== Motion Magic (Speed Limits) ====================
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 300.0;
  public static final double MOTION_MAGIC_ACCELERATION = 150.0;

  // ======================== Shooter Physical Constants ===========================
  // Degrees from horizontal
  public static final double HOOD_MAX = 41;
  public static final double HOOD_MIN = 27.5;

  public static final double WHEEL_DIAMETER = 0.1; // Example value in meters

  /** Range of error for the "isAtTargetVelocity" function */
  public static final AngularVelocity FLYWHEEL_TARGET_ERROR_RANGE = RotationsPerSecond.of(25);

  public static final Angle HOOD_TARGET_ERROR_RANGE = Degrees.of(2.5);

  public static final double JAM_DETECT_TIME = 1.0;

  /* Offsets (center of robot to center of bottom flywheel) */

  public static final Transform3d kLEFT_SHOOTER_OFFSET =
      new Transform3d(
          Units.inchesToMeters(8.785),
          Units.inchesToMeters(4.394),
          Units.inchesToMeters(20.789),
          Rotation3d.kZero);
  public static final Transform3d kRIGHT_SHOOTER_OFFSET =
      new Transform3d(
          Units.inchesToMeters(8.785),
          Units.inchesToMeters(-4.394),
          Units.inchesToMeters(20.789),
          Rotation3d.kZero);

  private ShooterConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
