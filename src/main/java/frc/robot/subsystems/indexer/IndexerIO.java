// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerInputs {
    public double motorCurrent = 0.0;
    public double motorVelocity = 0.0;
  }

  /**
   * Updates the set of logged inputs for the indexer subsystem.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(IndexerInputs inputs) {}

  public default void setIndexMotor(double speed) {}

  public default Current getIndexMotorCurrent() {
    return Amps.of(0);
  }

  public default AngularVelocity getIndexMotorVelocity() {
    return RotationsPerSecond.of(0);
  }

  public default void update() {}
}