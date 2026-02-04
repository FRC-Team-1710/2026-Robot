// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import frc.robot.constants.CanIdConstants;

@Logged
/** Creates a new IndexIOCTRE. */
public class IndexerIOCTRE implements IndexerIO {
  private final TalonFX IndexPrimary;
  private final TalonFX IndexSecondary;

  TalonFXConfiguration motorConfig;

  public IndexerIOCTRE() {
    this.IndexPrimary = new TalonFX(CanIdConstants.Indexer.INDEXER_MOTOR_PRIMARY);
    this.IndexSecondary = new TalonFX(CanIdConstants.Indexer.INDEXER_MOTOR_SECONDARY);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    IndexPrimary.getConfigurator().apply(motorConfig);
    IndexSecondary.getConfigurator().apply(motorConfig);
  }

  public void setIndexMotor(double speed) {
    IndexPrimary.set(speed);
    IndexSecondary.set(speed);
  }
}
