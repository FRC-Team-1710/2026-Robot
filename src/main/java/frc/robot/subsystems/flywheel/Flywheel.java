// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.FlywheelConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Flywheel extends SubsystemBase {

  // Main motor that spins the flywheel (device ID 21)
  protected final TalonFX leader = new TalonFX(21, TunerConstants.kCANBus);

  // Controller for spinning the flywheel at a target speed
  private final MotionMagicVelocityVoltage velocityOut = new MotionMagicVelocityVoltage(0);

  // How close the speed needs to be to count as "at target"
  private final AngularVelocity tolerance = RotationsPerSecond.of(FlywheelConstants.VELOCITY_TOLERANCE_RPS);

  // Configuration settings for the flywheel motor
  protected TalonFXConfiguration config = new TalonFXConfiguration();

  public Flywheel() {
    // Coast mode: Flywheel can spin freely by hand when disabled
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise spin
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Control values from FlywheelConstants
    config.Slot0.kS = FlywheelConstants.kS; // Static friction
    config.Slot0.kV = FlywheelConstants.kV; // Velocity feedforward
    config.Slot0.kP = FlywheelConstants.kP; // Proportional gain

    // Speed limits from FlywheelConstants
    config.MotionMagic.MotionMagicCruiseVelocity = FlywheelConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = FlywheelConstants.MOTION_MAGIC_ACCELERATION;

    // Apply configuration with retries
    if (TalonFXUtil.applyConfigWithRetries(leader, config, 2)) {
      Robot.telemetry().log("Flywheel/Config", true);
    } else {
      Robot.telemetry().log("Flywheel/Config", false);
    }
  }

  @Override
  public void periodic() {
    // No periodic updates needed - control is entirely feedforward/feedback
  }

  /**
   * Spin the flywheel at a specific speed.
   * Private to enforce Command-based control flow.
   *
   * @param velocity How fast to spin (rotations per second)
   */
  private void setVelocity(AngularVelocity velocity) {
    leader.setControl(velocityOut.withVelocity(velocity));
  }

  /**
   * Command to spin up the flywheel to shooting speed.
   *
   * @return Command that spins up the flywheel
   */
  public Command spinUp() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(FlywheelConstants.SHOOTING_SPEED_RPS)));
  }

  /**
   * Command to spin at amp scoring speed (slow and controlled).
   *
   * @return Command that spins flywheel at amp speed
   */
  public Command ampSpeed() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(FlywheelConstants.AMP_SPEED_RPS)));
  }

  /**
   * Command to spin at far shooting speed (fast).
   *
   * @return Command that spins flywheel at far speed
   */
  public Command farSpeed() {
    return runOnce(() -> setVelocity(RotationsPerSecond.of(FlywheelConstants.FAR_SHOOTING_SPEED_RPS)));
  }

  /**
   * Command to stop the flywheel.
   *
   * @return Command that stops the flywheel
   */
  public Command stopCommand() {
    return runOnce(() -> stop());
  }

  /**
   * Check if the flywheel has reached its target speed.
   *
   * @return true if close enough to target speed, false otherwise
   */
  public boolean isAtTarget() {
    return getVelocity().isNear(getTargetVelocity(), tolerance);
  }

  /**
   * Get how fast the flywheel is currently spinning.
   *
   * @return Current flywheel speed
   */
  public AngularVelocity getVelocity() {
    return leader.getVelocity().getValue();
  }

  /**
   * Get what speed the flywheel is trying to reach.
   *
   * @return Target flywheel speed
   */
  public AngularVelocity getTargetVelocity() {
    return velocityOut.getVelocityMeasure();
  }

  /**
   * Get the speed tolerance for "at target" checks.
   *
   * @return Speed tolerance
   */
  public AngularVelocity getTolerance() {
    return tolerance;
  }

  // Stop the flywheel motors (private to enforce Command-based control flow)
  private void stop() {
    leader.stopMotor();
  }
}