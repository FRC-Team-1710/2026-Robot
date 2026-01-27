package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class ShooterIOCTRE implements ShooterIO {

  private final MotionMagicVelocityVoltage m_velocityManager;

  private final TalonFX m_flyWheel;
  private final TalonFX m_flyWheel_Follower;
  private final TalonFX m_hood;

  public ShooterIOCTRE() {
    this.m_flyWheel = new TalonFX(CanIdConstants.Shooter.SHOOTER_MOTOR);
    this.m_flyWheel_Follower = new TalonFX(CanIdConstants.Shooter.SHOOTER_FOLLOWER_MOTOR);

    this.m_hood = new TalonFX(CanIdConstants.Shooter.HOOD_MOTOR);

    // Flywheel settings
    TalonFXConfiguration m_config = new TalonFXConfiguration();

    m_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_config.Slot0.kS = ShooterConstants.kS; // Static friction
    m_config.Slot0.kV = ShooterConstants.kV; // Velocity feedforward
    m_config.Slot0.kP = ShooterConstants.kP; // Proportional gain

    m_config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    m_config.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;

    TalonFXUtil.applyConfigWithRetries(this.m_flyWheel, m_config, 2);

    TalonFXUtil.applyConfigWithRetries(this.m_flyWheel_Follower, m_config, 2);

    this.m_velocityManager = new MotionMagicVelocityVoltage(0);
  }

  public void update() {}

  public void stop() {
    this.m_flyWheel.stopMotor();
    this.m_flyWheel_Follower.stopMotor();
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    this.m_flyWheel.setControl(this.m_velocityManager.withVelocity(velocity));
    this.m_flyWheel_Follower.setControl(this.m_velocityManager.withVelocity(velocity));
  }

  public AngularVelocity getVelocity() {
    return DegreesPerSecond.of(this.m_flyWheel.get());
  }

  public AngularVelocity getTargetVelocity() {
    return DegreesPerSecond.of(this.m_velocityManager.Velocity);
  }

  public void setHoodAngle(Angle angle) {
    this.m_hood.setPosition(angle);
  }

  public Angle getHoodAngle() {
    return this.m_hood.getPosition().getValue();
  }
}
