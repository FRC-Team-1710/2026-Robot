package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.FeederConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class FeederIOCTRE implements FeederIO {

  private final MotionMagicVelocityVoltage m_velocityManager;

  private final TalonFX m_feederLeft;
  private final TalonFX m_feederRight;

  public FeederIOCTRE() {
    this.m_feederLeft = new TalonFX(CanIdConstants.Feeder.FEEDER_MOTOR);
    this.m_feederRight = new TalonFX(CanIdConstants.Feeder.FEEDER_FOLLOWER_MOTOR);

    // Flywheel settings
    TalonFXConfiguration m_config = new TalonFXConfiguration();

    m_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_config.Slot0.kS = FeederConstants.kS;
    m_config.Slot0.kV = FeederConstants.kV;
    m_config.Slot0.kP = FeederConstants.kP;

    m_config.MotionMagic.MotionMagicCruiseVelocity = FeederConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    m_config.MotionMagic.MotionMagicAcceleration = FeederConstants.MOTION_MAGIC_ACCELERATION;

    TalonFXUtil.applyConfigWithRetries(this.m_feederLeft, m_config, 2);

    this.m_velocityManager = new MotionMagicVelocityVoltage(0);
    this.m_feederLeft.setControl(
        new Follower(CanIdConstants.Feeder.FEEDER_MOTOR, MotorAlignmentValue.Opposed));
  }

  public void update() {}

  public void setVelocity(AngularVelocity pVelocity) {
    this.m_feederRight.setControl(this.m_velocityManager.withVelocity(pVelocity));
  }
}
