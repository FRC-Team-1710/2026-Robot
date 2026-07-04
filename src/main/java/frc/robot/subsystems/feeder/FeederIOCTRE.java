package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.CanIdConstants;
import frc.robot.utils.TalonFXUtil;

/**
 * FeederIO implementation for CTRE TalonFX motor controllers.
 * This class provides the necessary methods to control the feeder motor using a TalonFX.
 */

public class FeederIOCTRE implements FeederIO {
  private final TalonFX m_feederMotor;

  private final BaseStatusSignal[] m_feederSignals;

  private final VoltageOut m_feederVoltageOutput = new VoltageOut(0).withEnableFOC(true);

  public FeederIOCTRE() {

    // Initialize the TalonFX motor controller for the feeder.

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
  public void update(double dtSeconds) { // Update the feeder's status signals.
    BaseStatusSignal.refreshAll(m_feederSignals);
  }

  /** {@inheritDoc} */
  public void setFeeder(double percent) { //sets the feeder motor output to a percentage of the maximum voltage (12V) based on the input percent value.
    this.m_feederMotor.setControl(m_feederVoltageOutput.withOutput(12 * percent));
  }
}
