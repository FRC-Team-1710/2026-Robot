// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface IndexerIO {
  public default void setIndexMotor(double speed) {}

  public default void updateVisual() {}

  public default void setSpeed(double speed) {}
}
