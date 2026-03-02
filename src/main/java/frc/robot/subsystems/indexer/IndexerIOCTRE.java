// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import frc.robot.constants.CanIdConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
/** Creates a new IndexIOCTRE. */
public class IndexerIOCTRE implements IndexerIO {
  @Logged(importance = Logged.Importance.CRITICAL)
  private final TalonFX m_IndexerMotor;

  @NotLogged TalonFXConfiguration motorConfig;

  @NotLogged private final BaseStatusSignal[] m_indexerSignals;

  @NotLogged
  private final DutyCycleOut m_indexerPercentOutput = new DutyCycleOut(0).withEnableFOC(true);

  public IndexerIOCTRE() {
    this.m_IndexerMotor = new TalonFX(CanIdConstants.Indexer.INDEXER_MOTOR);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_IndexerMotor.getConfigurator().apply(motorConfig);

    m_indexerSignals = TalonFXUtil.getBasicStatusSignals(m_IndexerMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_indexerSignals);

    m_IndexerMotor.optimizeBusUtilization();
  }

  public void update() {
    BaseStatusSignal.refreshAll(m_indexerSignals);
  }

  public void setIndexMotor(double speed) {
    m_IndexerMotor.setControl(m_indexerPercentOutput.withOutput(speed));
  }
}
