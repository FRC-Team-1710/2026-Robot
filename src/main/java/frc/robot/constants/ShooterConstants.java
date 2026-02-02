// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ShooterConstants {
  // ==================== PID Control Values ====================

  /** Static friction compensation */
  public static final double kS = 0.0;

  /** Velocity feedforward (predicts voltage needed for a speed) */
  public static final double kV = 0.125;

  /** Proportional gain (corrects speed errors) */
  public static final double kP = 0.0;

  // ==================== Current Limits ====================
  public static final int FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60;
  public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 80;

  public static final int HOOD_SUPPLY_CURRENT_LIMIT = 20;

  // ==================== Motion Magic (Speed Limits) ====================
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 100.0;
  public static final double MOTION_MAGIC_ACCELERATION = 1000.0;

  // ======================== Shooter Physical Constants ===========================
  public static final double WHEEL_DIAMETER = 0.1; // Example value in meters

  /** Range of error for the "isAtTargetVelocity" function */
  public static final AngularVelocity FLYWHEEL_TARGET_ERROR_RANGE = RotationsPerSecond.of(10);

  public static final Angle HOOD_TARGET_ERROR_RANGE = Degrees.of(2.5);

  private ShooterConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
