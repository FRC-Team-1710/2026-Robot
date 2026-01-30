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
  private final TalonFX m_flyWheelFollower;
  private final TalonFX m_hood;

  public ShooterIOCTRE() {
    this.m_flyWheel = new TalonFX(CanIdConstants.Shooter.SHOOTER_MOTOR);
    this.m_flyWheelFollower = new TalonFX(CanIdConstants.Shooter.SHOOTER_FOLLOWER_MOTOR);

    this.m_hood = new TalonFX(CanIdConstants.Shooter.HOOD_MOTOR);

    // Flywheel settings
    TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();

    FlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    FlywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    FlywheelConfig.Slot0.kS = ShooterConstants.kS; // Static friction
    FlywheelConfig.Slot0.kV = ShooterConstants.kV; // Velocity feedforward
    FlywheelConfig.Slot0.kP = ShooterConstants.kP; // Proportional gain

    FlywheelConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    FlywheelConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;

    FlywheelConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    FlywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
    FlywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    TalonFXUtil.applyConfigWithRetries(this.m_flyWheel, FlywheelConfig, 2);
    TalonFXUtil.applyConfigWithRetries(this.m_flyWheelFollower, FlywheelConfig, 2);

    // Hood Settings
    TalonFXConfiguration HoodConfig = new TalonFXConfiguration();

    HoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.HOOD_SUPPLY_CURRENT_LIMIT;
    HoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    TalonFXUtil.applyConfigWithRetries(this.m_hood, HoodConfig, 2);

    this.m_velocityManager = new MotionMagicVelocityVoltage(0);
  }

  public void update() {}

  public void stop() {
    this.m_flyWheel.stopMotor();
    this.m_flyWheelFollower.stopMotor();
  }

  public void setTargetVelocity(AngularVelocity velocity) {
    this.m_flyWheel.setControl(this.m_velocityManager.withVelocity(velocity));
    this.m_flyWheelFollower.setControl(this.m_velocityManager.withVelocity(velocity));
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
