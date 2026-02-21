// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

/**
 * Hardware abstraction interface for the Intake subsystem.
 *
 * <p>Implementations provide the concrete ways to control the intake's deployment (arm) and roller
 * motors, as well as to read back sensor values. Typical implementations are {@code IntakeIOCTRE}
 * (real hardware) and {@code IntakeIOSIM} (simulation).
 */
@Logged
public interface IntakeIO {
  /**
   * Command the deployment/arm to the requested angle setpoint.
   *
   * @param angle desired arm angle
   */
  public default void setAngle(Angle angle) {}

  /**
   * Sets the intake motor parameters.
   *
   * @param speed The speed of the intake roller motor
   * @param deploymentVelocity The desired maximum deployment velocity, in units of rotations per
   *     second.
   * @param deploymentAcceleration The desired maximum deployment acceleration, in units of
   *     rotations per second squared.
   */
  public default void setIntakeMotor(
      double speed, double deploymentVelocity, double deploymentAcceleration) {}

  /**
   * Called periodically to allow the IO implementation to refresh status signals, update internal
   * state, or sample sensors.
   */
  public default void update() {}

  /**
   * @return the current draw of the intake roller motor
   */
  @Logged(importance = Importance.DEBUG)
  public default Current getRollerCurrent() {
    return Amps.of(0);
  }

  /**
   * @return the angular velocity of the intake roller
   */
  @Logged(importance = Importance.DEBUG)
  public default AngularVelocity getRollerVelocity() {
    return RotationsPerSecond.of(0);
  }

  /**
   * @return the current draw of the deployment (arm) motor
   */
  @Logged(importance = Importance.DEBUG)
  public default Current getDeploymentCurrent() {
    return Amps.of(0);
  }
}
