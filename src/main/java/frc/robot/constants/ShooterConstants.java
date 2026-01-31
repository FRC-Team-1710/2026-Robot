// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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

  /** Range of error for the "isAtTargetVelocity" function */
  public static final double TARGET_ERROR_RANGE = 1;

  private ShooterConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
