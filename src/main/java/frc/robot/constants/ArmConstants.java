// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Constants for the Arm subsystem.
 *
 * <p>Centralizes all arm-related configuration including:
 * <ul>
 *   <li>Position setpoints (vertical, horizontal, scoring)
 *   <li>Motion constraints and tolerances
 *   <li>PID gains (TODO: needs tuning on hardware)
 * </ul>
 */
public final class ArmConstants {

  // ==================== Position Setpoints (in rotations) ====================

  /** Arm vertical position (stowed/safe transport) - 0.25 rotations */
  public static final double VERTICAL_POSITION_ROTATIONS = 0.25;

  /** Arm horizontal position (intake from ground) - 0.5 rotations */
  public static final double HORIZONTAL_POSITION_ROTATIONS = 0.5;

  /** Arm scoring position angle in degrees */
  public static final double SCORING_POSITION_DEGREES = 30.0;

  /** Arm scoring position in rotations */
  public static final double SCORING_POSITION_ROTATIONS = Units.degreesToRotations(SCORING_POSITION_DEGREES);

  /** Arm high scoring position angle in degrees (for far shots) */
  public static final double SCORING_HIGH_POSITION_DEGREES = 45.0;

  /** Arm high scoring position in rotations */
  public static final double SCORING_HIGH_POSITION_ROTATIONS =
      Units.degreesToRotations(SCORING_HIGH_POSITION_DEGREES);

  // ==================== Tolerances ====================

  /** Position tolerance in degrees for isAtTarget() checks */
  public static final double POSITION_TOLERANCE_DEGREES = 1.0;

  // ==================== PID Control Values ====================
  // TODO: CRITICAL - These control how the arm moves! Test on real robot first!
  // Start with these safe values: kG=0.2 (fights gravity), kS=0.2 (overcomes friction),
  //                               kP=160 (speed of correction), kD=30 (smoothness)
  // If the arm jerks or moves too fast, make these numbers smaller.

  /** Gravity feedforward gain */
  public static final double kG = 0.0; // NEEDS TUNING

  /** Static friction feedforward gain */
  public static final double kS = 0.0; // NEEDS TUNING

  /** Proportional gain */
  public static final double kP = 0.0; // NEEDS TUNING

  /** Derivative gain */
  public static final double kD = 0.0; // NEEDS TUNING

  // ==================== Motion Magic (Speed Limits) ====================
  // TODO: CRITICAL - Set how fast the arm can move!
  // Recommended starting values: Max Speed=2 rotations/second, Acceleration=4 rotations/second²

  /** Motion Magic cruise velocity in rotations per second */
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0.0; // NEEDS SETTING

  /** Motion Magic acceleration in rotations per second² */
  public static final double MOTION_MAGIC_ACCELERATION = 0.0; // NEEDS SETTING

  private ArmConstants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
