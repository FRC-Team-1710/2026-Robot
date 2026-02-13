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
  private final TalonFX IndexMotor;

  TalonFXConfiguration motorConfig;

  private final BaseStatusSignal[] m_indexerSignals;

  public IndexerIOCTRE() {
    this.IndexMotor = new TalonFX(CanIdConstants.Indexer.INDEXER_MOTOR);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0625;
    motorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.0625;

    IndexMotor.getConfigurator().apply(motorConfig);

    m_indexerSignals = TalonFXUtil.getBasicStatusSignals(IndexMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_indexerSignals);

    IndexMotor.optimizeBusUtilization();
  }

  public void update() {
    BaseStatusSignal.refreshAll(m_indexerSignals);
  }

  public void setSpeed(double speed) {
    IndexMotor.set(speed);
  }
}
