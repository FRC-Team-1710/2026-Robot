// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Arm extends SubsystemBase {
  // Connect to the "canivore" CAN bus (communication network for motors)
  private final CANBus canivore = new CANBus("canivore");

  // Main motor that moves the arm (device ID 31)
  protected final TalonFX leader = new TalonFX(31, canivore);
  // Sensor that tells us the arm's exact angle (device ID 32)
  protected final CANcoder encoder = new CANcoder(32, canivore);

  // Configuration settings for the arm motor
  protected TalonFXConfiguration config = new TalonFXConfiguration();

  // Controller for moving the arm to specific positions
  private final MotionMagicVoltage positionOut = new MotionMagicVoltage(0);

  // How close the arm needs to be to count as "at target" (in degrees)
  private final Angle tolerance = Degrees.of(ArmConstants.POSITION_TOLERANCE_DEGREES);

  public Arm() {
    // Coast mode: Motor can be moved by hand when disabled (easier for testing)
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise rotation
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Automatically fights gravity using math

    // Control values from ArmConstants (TODO: CRITICAL - Tune these on the real robot!)
    config.Slot0.kG = ArmConstants.kG; // Gravity compensation
    config.Slot0.kS = ArmConstants.kS; // Static friction
    config.Slot0.kP = ArmConstants.kP; // Proportional gain (speed of correction)
    config.Slot0.kD = ArmConstants.kD; // Derivative gain (smoothness)

    // Motion limits from ArmConstants (TODO: CRITICAL - Set non-zero values!)
    config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MOTION_MAGIC_CRUISE_VELOCITY; // Max speed
    config.MotionMagic.MotionMagicAcceleration = ArmConstants.MOTION_MAGIC_ACCELERATION; // How fast to speed up
    // Tell the motor to use the CANcoder sensor for position measurements
    config.Feedback.withRemoteCANcoder(encoder);

    // Apply configuration with retries
    if (TalonFXUtil.applyConfigWithRetries(leader, config)) {
      Robot.telemetry().log("Arm/Config", true);
    } else {
      Robot.telemetry().log("Arm/Config", false);
    }
  }

  @Override
  public void periodic() {
    // No periodic updates needed - control is entirely feedforward/feedback
  }

  /**
   * Move the arm to a specific angle.
   *
   * @param position Where to move the arm (in rotations)
   */
  private void setPosition(double position) {
    // Tell the motor to move to this position
    leader.setControl(positionOut.withPosition(position));
  }

  /**
   * Command to move the arm to vertical position (safe for transport).
   *
   * @return Command that moves arm vertical
   */
  public Command vertical() {
    return runOnce(() -> setPosition(ArmConstants.VERTICAL_POSITION_ROTATIONS));
  }

  /**
   * Command to move the arm to horizontal position (for ground intake).
   *
   * @return Command that moves arm horizontal
   */
  public Command horizontal() {
    return runOnce(() -> setPosition(ArmConstants.HORIZONTAL_POSITION_ROTATIONS));
  }

  /**
   * Command to move the arm to scoring position.
   *
   * @return Command that moves arm to scoring angle
   */
  public Command scoringPosition() {
    return runOnce(() -> setPosition(ArmConstants.SCORING_POSITION_ROTATIONS));
  }

  /**
   * Command to move the arm to high scoring position (far shots).
   *
   * @return Command that moves arm to high scoring angle
   */
  public Command scoringHighPosition() {
    return runOnce(() -> setPosition(ArmConstants.SCORING_HIGH_POSITION_ROTATIONS));
  }

  /**
   * Command to stop the arm motor.
   *
   * @return Command that stops the arm
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  // Stop the arm motor (private to enforce Command-based control flow)
  private void stop() {
    leader.stopMotor();
  }

  /**
   * Check if the arm has reached its target position.
   *
   * @return true if close enough to target, false otherwise
   */
  public boolean isAtTarget() {
    return getPosition().isNear(getTargetPosition(), tolerance);
  }

  /**
   * Get where the arm currently is.
   *
   * @return Current arm angle
   */
  public Angle getPosition() {
    return encoder.getPosition().getValue();
  }

  /**
   * Get where the arm is trying to move to.
   *
   * @return Target arm angle
   */
  public Angle getTargetPosition() {
    return positionOut.getPositionMeasure();
  }

  /**
   * Get the position tolerance for "at target" checks.
   *
   * @return Position tolerance
   */
  public Angle getTolerance() {
    return tolerance;
  }
}