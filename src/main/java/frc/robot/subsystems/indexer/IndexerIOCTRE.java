// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.CanIdConstants;
import frc.robot.utils.TalonFXUtil;

/** Creates a new IndexerIOCTRE. */
public class IndexerIOCTRE implements IndexerIO {
  private final TalonFX m_indexerMotor;

  TalonFXConfiguration motorConfig;

  private final BaseStatusSignal[] m_indexerSignals;

  private final VoltageOut m_indexerVoltageOutput = new VoltageOut(0).withEnableFOC(true);

  public IndexerIOCTRE() {
    this.m_indexerMotor = new TalonFX(CanIdConstants.Indexer.INDEXER_MOTOR);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_indexerMotor.getConfigurator().apply(motorConfig);

    m_indexerSignals = TalonFXUtil.getBasicStatusSignals(m_indexerMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_indexerSignals);

    m_indexerMotor.optimizeBusUtilization();
  }

  public void update() {
    BaseStatusSignal.refreshAll(m_indexerSignals);
  }

  public void updateInputs(IndexerInputs inputs) {
    inputs.motorCurrent = m_indexerMotor.getStatorCurrent(false).getValue().in(Amps);
    inputs.motorVelocity = m_indexerMotor.getVelocity(false).getValue().in(RotationsPerSecond);
  }

  public void setIndexMotor(double speed) {
    m_indexerMotor.setControl(m_indexerVoltageOutput.withOutput(speed * 12));
  }
}