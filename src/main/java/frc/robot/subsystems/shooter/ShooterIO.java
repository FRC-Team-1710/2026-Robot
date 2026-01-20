// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
  public default void stop() {}

  public default void setVelocity(AngularVelocity velocity) {}

  public default void setHoodAngle(Angle angle) {}

  public default Angle getHoodAngle() {
    return null;
  }
}
