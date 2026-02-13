// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import frc.robot.constants.CanIdConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
/** Creates a new IndexIOCTRE. */
public class IndexerIOCTRE implements IndexerIO {
  private final TalonFX IndexerAlpha;
  private final TalonFX IndexerBeta;

  TalonFXConfiguration motorConfig;

  private final BaseStatusSignal[] m_baseStatusSignals;

  public IndexerIOCTRE() {
    this.IndexerAlpha = new TalonFX(CanIdConstants.Indexer.INDEXER_MOTOR_ALPHA);
    this.IndexerBeta = new TalonFX(CanIdConstants.Indexer.INDEXER_MOTOR_BETA);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    IndexerAlpha.getConfigurator().apply(motorConfig);
    IndexerBeta.getConfigurator().apply(motorConfig);

    m_baseStatusSignals = TalonFXUtil.getBasicStatusSignals(IndexMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_baseStatusSignals);

    IndexMotor.optimizeBusUtilization();
  }

  public void update() {
    BaseStatusSignal.refreshAll(m_baseStatusSignals);
  }

  public void setIndexMotor(double speed) {
    IndexerAlpha.set(speed);
    IndexerBeta.set(speed);
  }
}
