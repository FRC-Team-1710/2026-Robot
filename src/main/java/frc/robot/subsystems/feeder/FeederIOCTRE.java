package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.SubsystemConstants.FeederConstants;
import frc.robot.utils.TalonFXUtil;

public class FeederIOCTRE implements FeederIO {

  @NotLogged private final TalonFX m_feederMotor;

  @NotLogged private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true);

  public FeederIOCTRE() {
    m_feederMotor = new TalonFX(CanIdConstants.Feeder.FEEDER_MOTOR);

    TalonFXUtil.applyConfig(FeederConstants.Software.Config.kConfig, m_feederMotor);
    TalonFXUtil.optimizeForBasicStatusSignals(m_feederMotor);
  }

  /** {@inheritDoc} */
  @Override
  public void setVoltage(Voltage voltage) {
    m_feederMotor.setControl(m_voltageOut.withOutput(voltage));
  }
}
