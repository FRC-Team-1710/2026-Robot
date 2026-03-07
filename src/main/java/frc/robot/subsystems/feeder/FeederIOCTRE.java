package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
  private final TalonFX m_feederLeft;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_feederRight;

  @NotLogged private final BaseStatusSignal[] m_feederSignals;

  @NotLogged private final BaseStatusSignal[] m_feederFollowerSignals;

  @NotLogged
  private final DutyCycleOut m_leftPercentOutput = new DutyCycleOut(0).withEnableFOC(true);

  @NotLogged
  private final DutyCycleOut m_rightPercentOutput = new DutyCycleOut(0).withEnableFOC(true);

  public FeederIOCTRE() {
    this.m_feederRight = new TalonFX(CanIdConstants.Feeder.FEEDER_MOTOR);
    this.m_feederLeft = new TalonFX(CanIdConstants.Feeder.FEEDER_FOLLOWER_MOTOR);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXUtil.applyConfigWithRetries(this.m_feederLeft, config, 2);
    TalonFXUtil.applyConfigWithRetries(this.m_feederRight, config, 2);

    m_feederSignals = TalonFXUtil.getBasicStatusSignals(m_feederLeft);
    m_feederFollowerSignals = TalonFXUtil.getBasicStatusSignals(m_feederRight);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_feederSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_feederFollowerSignals);

    m_feederLeft.optimizeBusUtilization();
    m_feederRight.optimizeBusUtilization();
  }

  public void update(double dtSeconds) {
    BaseStatusSignal.refreshAll(m_feederSignals);
    BaseStatusSignal.refreshAll(m_feederFollowerSignals);
  }

  public void setLeft(double percent) {
    this.m_feederLeft.setControl(m_leftPercentOutput.withOutput(-percent));
  }

  public void setRight(double percent) {
    this.m_feederRight.setControl(m_rightPercentOutput.withOutput(percent));
  }
}
