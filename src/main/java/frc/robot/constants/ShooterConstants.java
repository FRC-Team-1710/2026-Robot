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
  public static final double kFlyS = 0.0; // TODO: Tune this

  /** Velocity feedforward (predicts voltage needed for a speed) */
  public static final double kFlyV = 0.0; // TODO: Tune this

  /** Proportional gain (corrects speed errors) */
  public static final double kFlyP = 0.0; // TODO: Tune this

  /** Derivative gain (corrects acceleration errors) */
  public static final double kFlyD = 0.0; // TODO: Tune this

  /** Static friction compensation */
  public static final double kHoodS = 0.0; // TODO: Tune this

  /** Gravity feedforward (predicts voltage needed to keep at current angle) */
  public static final double kHoodG = 0.0; // TODO: Tune this

  /** Velocity feedforward (predicts voltage needed for a speed) */
  public static final double kHoodV = 0.0; // TODO: Tune this

  /** Proportional gain (corrects position errors) */
  public static final double kHoodP = 0.0; // TODO: Tune this

  /** Derivative gain (corrects speed errors) */
  public static final double kHoodD = 0.0; // TODO: Tune this

  // ==================== Current Limits ====================
  public static final int FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60;
  public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 80;

  // ==================== Motion Magic (Speed Limits) ====================
  public static final double FLYWHEEL_MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // TODO: Tune this
  public static final double FLYWHEEL_MOTION_MAGIC_ACCELERATION = 0.0; // TODO: Tune this

  public static final double HOOD_MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // TODO: Tune this
  public static final double HOOD_MOTION_MAGIC_ACCELERATION = 0.0; // TODO: Tune this

  // ======================== Shooter Physical Constants ===========================
  // Degrees from horizontal
  public static final double HOOD_MAX = 38.3; // 0.6684611 radians
  public static final double HOOD_MIN = 11.0; // 0.191986 radians

  /** Range of error for the "isAtTargetVelocity" function */
  public static final AngularVelocity FLYWHEEL_TARGET_ERROR_RANGE = RotationsPerSecond.of(25);

  /** Range of error for the "isAtTargetAngle" function */
  public static final Angle HOOD_TARGET_ERROR_RANGE = Degrees.of(2.5);

  /** Offset (center of robot to center of bottom flywheel) */
  public static final Transform3d kSHOOTER_OFFSET =
      new Transform3d(
          Units.inchesToMeters(8.50),
          Units.inchesToMeters(0),
          Units.inchesToMeters(20.05),
          Rotation3d.kZero);

  private ShooterConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
