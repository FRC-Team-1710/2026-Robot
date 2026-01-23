// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;

@Logged
public interface IntakeIO {
  public default void setAngle(Angle angle) {}

  public default void setIntakeMotor(double speed) {}
}
