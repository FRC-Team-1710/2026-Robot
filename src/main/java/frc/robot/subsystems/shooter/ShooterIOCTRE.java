package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.FuelSim;
import frc.robot.utils.TalonFXUtil;

@Logged
public class ShooterIOCTRE implements ShooterIO {

  private final MotionMagicVelocityVoltage m_velocityManager;
  private final PositionVoltage m_positionManager;

  private final TalonFX m_flyWheel;
  private final TalonFX m_flyWheelFollower;
  private final TalonFX m_hood;
  private final TalonFX m_hoodFollower;

  private final BaseStatusSignal[] m_flyWheelSignals;
  private final BaseStatusSignal[] m_flyWheelFollowerSignals;
  private final BaseStatusSignal[] m_hoodSignals;
  private final BaseStatusSignal[] m_hoodFollowerSignals;

  private final DigitalInput m_breamBreaker;
  private final DigitalInput m_breamBreakerFollower;

  public ShooterIOCTRE() {
    this.m_flyWheel = new TalonFX(CanIdConstants.Shooter.SHOOTER_MOTOR);
    this.m_flyWheelFollower = new TalonFX(CanIdConstants.Shooter.SHOOTER_FOLLOWER_MOTOR);

    this.m_hood = new TalonFX(CanIdConstants.Shooter.HOOD_MOTOR);
    this.m_hoodFollower = new TalonFX(CanIdConstants.Shooter.HOOD_FOLLOWER_MOTOR);

    this.m_breamBreaker = new DigitalInput(0);
    this.m_breamBreakerFollower = new DigitalInput(1);

    // Flywheel settings
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelConfig.Slot0.kS = ShooterConstants.kS; // Static friction
    flywheelConfig.Slot0.kV = ShooterConstants.kV; // Velocity feedforward
    flywheelConfig.Slot0.kP = ShooterConstants.kP; // Proportional gain

    flywheelConfig.MotorOutput.PeakForwardDutyCycle = 1;
    flywheelConfig.MotorOutput.PeakReverseDutyCycle = 0;

    flywheelConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    flywheelConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;

    flywheelConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

    TalonFXUtil.applyConfigWithRetries(this.m_flyWheel, flywheelConfig, 2);
    TalonFXUtil.applyConfigWithRetries(this.m_flyWheelFollower, flywheelConfig, 2);

    // Hood Settings
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.Slot0.kP = 75;

    hoodConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.HOOD_SUPPLY_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodConfig.Feedback.SensorToMechanismRatio = 13.2;

    TalonFXUtil.applyConfigWithRetries(this.m_hood, hoodConfig, 2);
    TalonFXUtil.applyConfigWithRetries(this.m_hoodFollower, hoodConfig, 2);

    this.m_velocityManager = new MotionMagicVelocityVoltage(0).withSlot(0);
    this.m_positionManager = new PositionVoltage(0);

    m_flyWheelSignals = TalonFXUtil.getBasicStatusSignals(m_flyWheel);
    m_flyWheelFollowerSignals = TalonFXUtil.getBasicStatusSignals(m_flyWheelFollower);
    m_hoodSignals = TalonFXUtil.getBasicStatusSignals(m_hood);
    m_hoodFollowerSignals = TalonFXUtil.getBasicStatusSignals(m_hoodFollower);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_flyWheelSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_flyWheelFollowerSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_hoodSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_hoodFollowerSignals);

    m_flyWheel.optimizeBusUtilization();
    m_flyWheelFollower.optimizeBusUtilization();
    m_hood.optimizeBusUtilization();
    m_hoodFollower.optimizeBusUtilization();

    m_hood.setPosition(Degrees.of(ShooterConstants.HOOD_MIN));
    m_hoodFollower.setPosition(Degrees.of(ShooterConstants.HOOD_MIN));
  }

  public void update(double dtSeconds) {
    BaseStatusSignal.refreshAll(m_flyWheelSignals);
    BaseStatusSignal.refreshAll(m_flyWheelFollowerSignals);
    BaseStatusSignal.refreshAll(m_hoodSignals);
    BaseStatusSignal.refreshAll(m_hoodFollowerSignals);
  }

  public void stop() {
    this.m_flyWheel.stopMotor();
    this.m_flyWheelFollower.stopMotor();
  }

  public void setTargetVelocity(AngularVelocity pVelocity) {
    this.m_flyWheel.setControl(this.m_velocityManager.withVelocity(pVelocity));
    this.m_flyWheelFollower.setControl(this.m_velocityManager.withVelocity(pVelocity));
  }

  public AngularVelocity getVelocity() {
    return DegreesPerSecond.of(this.m_flyWheel.get());
  }

  public AngularVelocity getTargetVelocity() {
    return DegreesPerSecond.of(this.m_velocityManager.Velocity);
  }

  public void setHoodAngle(Angle pAngle) {
    Angle ClampedAngle =
        Degrees.of(
            MathUtil.clamp(
                pAngle.magnitude(), ShooterConstants.HOOD_MIN, ShooterConstants.HOOD_MAX));
    this.m_hood.setControl(m_positionManager.withPosition(ClampedAngle));
    this.m_hoodFollower.setControl(m_positionManager.withPosition(ClampedAngle));
  }

  public Angle getHoodAngle() {
    return this.m_hood.getPosition().getValue();
  }

  public boolean hasBreakerBroke() {
    return this.m_breamBreaker.get();
  }

  public boolean hasBreakerFollowerBroke() {
    return this.m_breamBreakerFollower.get();
  }

  public void setFuelSim(FuelSim fuelSim) {
    // CTRE implementation does not use FuelSim
  }
}
