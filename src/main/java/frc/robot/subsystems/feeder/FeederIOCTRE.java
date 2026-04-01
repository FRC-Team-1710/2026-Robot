package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import frc.robot.constants.CanIdConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class FeederIOCTRE implements FeederIO {

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_feederMotor;

  @NotLogged private final BaseStatusSignal[] m_feederSignals;

  @NotLogged private final VoltageOut m_leftVoltageOutput = new VoltageOut(0).withEnableFOC(true);

  @NotLogged private final VoltageOut m_rightVoltageOutput = new VoltageOut(0).withEnableFOC(true);

  public FeederIOCTRE() {
    this.m_feederMotor = new TalonFX(CanIdConstants.Feeder.FEEDER_MOTOR);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXUtil.applyConfigWithRetries(this.m_feederMotor, config, 2);

    m_feederSignals = TalonFXUtil.getBasicStatusSignals(m_feederMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_feederSignals);

    m_feederMotor.optimizeBusUtilization();
  }

  /** {@inheritDoc} */
  public void update(double dtSeconds) {
    BaseStatusSignal.refreshAll(m_feederSignals);
  }

  /** {@inheritDoc} */
  public void set(double percent) {
    this.m_feederMotor.setControl(m_leftVoltageOutput.withOutput(-12 * percent));
  }
}
