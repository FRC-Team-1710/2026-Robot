package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.SubsystemConstants.IndexerConstants;
import frc.robot.utils.TalonFXUtil;

public class IndexerIOCTRE implements IndexerIO {

  private final TalonFX m_indexerMotor;

  private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true);

  public IndexerIOCTRE() {
    m_indexerMotor = new TalonFX(CanIdConstants.Indexer.INDEXER_MOTOR);

    TalonFXUtil.applyConfig(IndexerConstants.Software.Config.kConfig, m_indexerMotor);
    TalonFXUtil.optimizeForBasicStatusSignals(m_indexerMotor);
  }

  /** {@inheritDoc} */
  @Override
  public void setVoltage(Voltage voltage) {
    m_indexerMotor.setControl(m_voltageOut.withOutput(voltage));
  }
}
