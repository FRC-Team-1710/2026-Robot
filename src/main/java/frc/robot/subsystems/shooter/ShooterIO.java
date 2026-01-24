// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

@Logged
public interface ShooterIO {
  public default void update() {}

  public default void stop() {}

  public default void setTargetVelocity(AngularVelocity velocity) {}

  public default AngularVelocity getVelocity() {
    return null;
  }

  public default AngularVelocity getTargetVelocity() {
    return null;
  }

  public default void setHoodAngle(Angle angle) {}

  public default Angle getHoodAngle() {
    return null;
  }
}
