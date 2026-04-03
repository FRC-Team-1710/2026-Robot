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

  public default void setLeftTargetVelocity(AngularVelocity pVelocity) {}

  public default void setRightTargetVelocity(AngularVelocity pVelocity) {}

  // Per-motor helper functions: Left Motor, Left Follower Motor, Right Motor, Right Follower Motor
  public default void setLeftMotorTargetVelocity(AngularVelocity pVelocity) {
    setLeftTargetVelocity(pVelocity);
  }

  public default void setLeftFollowerTargetVelocity(AngularVelocity pVelocity) {
    setLeftTargetVelocity(pVelocity);
  }

  public default void setRightMotorTargetVelocity(AngularVelocity pVelocity) {
    setRightTargetVelocity(pVelocity);
  }

  public default void setRightFollowerTargetVelocity(AngularVelocity pVelocity) {
    setRightTargetVelocity(pVelocity);
  }

  public default AngularVelocity getLeftVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default AngularVelocity getRightVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default AngularVelocity getLeftMotorVelocity() {
    return getLeftVelocity();
  }

  public default AngularVelocity getLeftFollowerVelocity() {
    return getLeftVelocity();
  }

  public default AngularVelocity getRightMotorVelocity() {
    return getRightVelocity();
  }

  public default AngularVelocity getRightFollowerVelocity() {
    return getRightVelocity();
  }

  public default AngularVelocity getLeftTargetVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default AngularVelocity getRightTargetVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default void setLeftHoodTarget(Angle pAngle) {}

  public default void setRightHoodTarget(Angle pAngle) {}

  public default Angle getLeftHoodPosition() {
    return Degrees.of(0);
  }

  public default Angle getRightHoodPosition() {
    return Degrees.of(0);
  }

  public default Angle getHoodPosition() {
    return getLeftHoodPosition();
  }

  public default boolean hasBreakerLeftBroke() {
    return false;
  }

  public default boolean hasBreakerRightBroke() {
    return false;
  }

  public default void setFuelSim(FuelSim fuelSim) {}
}
