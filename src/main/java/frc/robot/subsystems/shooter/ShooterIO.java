// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.FuelSim;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    public double flywheelVelocity = 0.0;
    public double hoodPosition = 0.0;
    public boolean setpointReferenceVelocityIsZero = false;
  }

  /**
   * Updates the set of logged inputs for the shooter subsystem.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(ShooterInputs inputs) {}

  /**
   * Updates cached inputs and simulation state.
   *
   * @param pDTSeconds loop delta time in seconds
   */
  public default void update(double pDTSeconds) {}

  /** Stops all shooter outputs. */
  public default void stop() {}

  /**
   * Sets the flywheel velocity target for the shooter.
   *
   * @param pVelocity target flywheel angular velocity
   */
  public default void setTargetVelocity(AngularVelocity pVelocity) {}

  /**
   * Returns the current flywheel velocity measurement.
   *
   * @return current flywheel angular velocity
   */
  public default AngularVelocity getVelocity() {
    return RotationsPerSecond.of(0);
  }

  /**
   * Sets the hood target angle.
   *
   * @param pAngle target hood angle
   */
  public default void setHoodTarget(Angle pAngle) {}

  /**
   * Returns the current hood angle measurement.
   *
   * @return current hood position angle
   */
  public default Angle getHoodPosition() {
    return Degrees.of(0);
  }

  /** Returns the closed loop reference slope == 0 */
  public default boolean getSetpointReferenceVelocityIsZero() {
    return false;
  }

  /**
   * Injects the fuel simulator reference for simulation-backed IO implementations.
   *
   * @param fuelSim shooter fuel simulator
   */
  public default void setFuelSim(FuelSim fuelSim) {}
}