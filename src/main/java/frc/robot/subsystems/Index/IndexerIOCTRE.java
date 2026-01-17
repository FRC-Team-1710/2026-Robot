// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Index;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;

@Logged
public class IndexerIOCTRE implements IndexerIO {
  /** Creates a new IndexIOCTR. */
  private final TalonFX IndexMotor;

  TalonFXConfiguration motorConfig;

  public IndexerIOCTRE() {
    this.IndexMotor = new TalonFX(31);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    IndexMotor.getConfigurator().apply(motorConfig);
  }

  public void setIndexMotor(double speed) {
    IndexMotor.set(speed);
  }
}
