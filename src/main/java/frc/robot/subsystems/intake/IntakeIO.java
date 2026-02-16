// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

@Logged
public interface IntakeIO {
  public default void setAngle(Angle angle) {}

  /**
   * Sets the intake motor parameters.
   *
   * @param speed The speed of the intake roller motor, typically in the range [-1.0, 1.0].
   * @param deploymentVelocity The desired deployment velocity, in units of rotations per second.
   * @param deploymentAcceleration The desired deployment acceleration, in units of rotations per
   *     second squared.
   */
  public default void setIntakeMotor(
      double speed, double deploymentVelocity, double deploymentAcceleration) {}

  public default void update() {}

  public default Current getRollerCurrent() {
    return Amps.of(0);
  }

  public default AngularVelocity getRollerVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default Current getDeploymentCurrent() {
    return Amps.of(0);
  }
}
