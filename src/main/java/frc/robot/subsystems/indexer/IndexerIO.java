// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

@Logged
public interface IndexerIO {
  public default void setIndexMotor(double speed) {}

  public default Current getIndexMotorCurrent() {
    return Amps.of(0);
  }

  public default AngularVelocity getIndexMotorVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default void updateVisual() {}
}
