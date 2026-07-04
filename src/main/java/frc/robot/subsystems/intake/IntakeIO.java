// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for the Intake subsystem.
 *
 * <p>Implementations provide the concrete ways to control the intake's deployment (arm) and roller
 * motors, as well as to read back sensor values. Typical implementations are {@code IntakeIOCTRE}
 * (real hardware) and {@code IntakeIOSIM} (simulation).
 */
public interface IntakeIO {
  @AutoLog
  public static class IntakeInputs {
    public double rollerCurrent = 0.0;
    public double followerCurrent = 0.0;
    public double rollerVelocity = 0.0;
    public double followerVelocity = 0.0;
    public double deploymentCurrent = 0.0;
  }

  /**
   * Updates the set of logged inputs for the intake subsystem.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(IntakeInputs inputs) {}

  /**
   * Command the deployment/arm to the requested angle setpoint.
   *
   * @param angle desired arm angle
   * @param velocity desired
   * @param acceleration desired
   */
  public default void setAngle(Angle angle, double velocity, double acceleration) {}

  /** Returns the closed loop reference slope == 0 */
  public default boolean getSetpointReferenceVelocityIsZero() {
    return false;
  }

  /**
   * Sets the intake motor parameters.
   *
   * @param speed The speed of the intake roller motor
   */
  public default void setIntakeMotor(double speed) {}

  /**
   * Called periodically to allow the IO implementation to refresh status signals, update internal
   * state, or sample sensors.
   */
  public default void update() {}

  /**
   * @return the current draw of the intake roller motor
   */
  public default Current getRollerCurrent() {
    return Amps.of(0);
  }

  /**
   * @return the current draw of the intake follower motor
   */
  public default Current getFollowerCurrent() {
    return Amps.of(0);
  }

  /**
   * @return the angular velocity of the intake roller
   */
  public default AngularVelocity getRollerVelocity() {
    return RotationsPerSecond.of(0);
  }

  /**
   * @return the angular velocity of the intake follower motor
   */
  public default AngularVelocity getFollowerVelocity() {
    return RotationsPerSecond.of(0);
  }

  /**
   * @return the current draw of the deployment (arm) motor
   */
  public default Current getDeploymentCurrent() {
    return Amps.of(0);
  }
}