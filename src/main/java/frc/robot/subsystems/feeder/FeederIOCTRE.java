package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import frc.robot.constants.CanIdConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class FeederIOCTRE implements FeederIO {

  private final TalonFX m_feederLeft;
  private final TalonFX m_feederRight;

  public FeederIOCTRE() {
    this.m_feederLeft = new TalonFX(CanIdConstants.Feeder.FEEDER_MOTOR);
    this.m_feederRight = new TalonFX(CanIdConstants.Feeder.FEEDER_FOLLOWER_MOTOR);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXUtil.applyConfigWithRetries(this.m_feederLeft, config, 2);

    this.m_feederLeft.setControl(
        new Follower(CanIdConstants.Feeder.FEEDER_MOTOR, MotorAlignmentValue.Opposed));
  }

  public void update() {}

  public void setFeeder(double percent) {
    this.m_feederRight.set(percent);
  }
}
