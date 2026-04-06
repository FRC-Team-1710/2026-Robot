// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.FuelSim;

@Logged
public interface ShooterIO {
  public default void update(double pDTSeconds) {}

  public default void stop() {}

  public default void setTargetVelocity(AngularVelocity pVelocity) {}

  public default void setRightTargetVelocity(AngularVelocity pVelocity) {}

  public default AngularVelocity getAverageVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default AngularVelocity getRightTargetVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default void setHoodTarget(Angle pAngle) {}

  public default Angle getHoodPosition() {
    return Degrees.of(0);
  }

  public default boolean hasBreakerLeftBroke() {
    return false;
  }

  public default boolean hasBreakerRightBroke() {
    return false;
  }

  public default void setFuelSim(FuelSim fuelSim) {}
}
