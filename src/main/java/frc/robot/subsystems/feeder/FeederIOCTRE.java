package frc.robot.subsystems.feeder;

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

public class FeederIOCTRE implements FeederIO {

  private final TalonFX m_feederMotor;

  private final BaseStatusSignal[] m_feederSignals;

  private final VoltageOut m_feederVoltageOutput = new VoltageOut(0).withEnableFOC(true);

  public FeederIOCTRE() {
    this.m_feederMotor = new TalonFX(CanIdConstants.Feeder.FEEDER_MOTOR);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Requirement: Counterclockwise is forward/feeding direction.
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 160;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    TalonFXUtil.applyConfigWithRetries(this.m_feederMotor, config, 2);

    m_feederSignals = TalonFXUtil.getBasicStatusSignals(m_feederMotor);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_feederSignals);

    m_feederMotor.optimizeBusUtilization();
  }

  /** {@inheritDoc} */
  public void update(double dtSeconds) {
    BaseStatusSignal.refreshAll(m_feederSignals);
  }

  public void updateInputs(FeederInputs inputs) {
    inputs.motorVelocity = m_feederMotor.getVelocity(false).getValue().in(RotationsPerSecond);
    inputs.motorCurrent = m_feederMotor.getStatorCurrent(false).getValue().in(Amps);
  }

  /** {@inheritDoc} */
  public void setFeeder(double percent) {
    this.m_feederMotor.setControl(m_feederVoltageOutput.withOutput(12 * percent));
  }
}