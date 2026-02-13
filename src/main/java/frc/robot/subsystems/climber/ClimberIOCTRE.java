// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.CanIdConstants;

@Logged
public class ClimberIOCTRE implements ClimberIO {
  private TalonFX m_climberMotor;

  public ClimberIOCTRE() {
    m_climberMotor = new TalonFX(CanIdConstants.Climber.CLIMBER_MOTOR);
  }

  @Override
  public void setSpeed(double speed, double dtSeconds) {
    m_climberMotor.set(speed);
  }

  public double getDegrees() {
    return Units.rotationsToDegrees(m_climberMotor.getRotorPosition().getValueAsDouble());
  }
}
