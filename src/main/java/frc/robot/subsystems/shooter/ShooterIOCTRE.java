package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
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

  @NotLogged private final MotionMagicVelocityVoltage m_leftVelocityRequest;
  @NotLogged private final MotionMagicVelocityVoltage m_rightVelocityRequest;

  @NotLogged private final PositionVoltage m_leftPositionRequest;
  @NotLogged private final PositionVoltage m_rightPositionRequest;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_leftFlywheel;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_rightFlywheel;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_leftHood;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_rightHood;

  @NotLogged private final BaseStatusSignal[] m_leftFlywheelSignals;
  @NotLogged private final BaseStatusSignal[] m_rightFlywheelSignals;
  @NotLogged private final BaseStatusSignal[] m_leftHoodSignals;
  @NotLogged private final BaseStatusSignal[] m_rightHoodSignals;

  @Logged(importance = Importance.INFO)
  private final DigitalInput m_leftBreamBreaker;

  @Logged(importance = Importance.INFO)
  private final DigitalInput m_rightBreamBreaker;

  public ShooterIOCTRE() {
    this.m_leftFlywheel = new TalonFX(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR);
    this.m_rightFlywheel = new TalonFX(CanIdConstants.Shooter.SHOOTER_RIGHT_MOTOR);

    this.m_leftHood = new TalonFX(CanIdConstants.Shooter.HOOD_LEFT_MOTOR);
    this.m_rightHood = new TalonFX(CanIdConstants.Shooter.HOOD_RIGHT_MOTOR);

    this.m_leftBreamBreaker = new DigitalInput(0);
    this.m_rightBreamBreaker = new DigitalInput(1);

    // Flywheel settings
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelConfig.Slot0.kS = ShooterConstants.kS;
    flywheelConfig.Slot0.kV = ShooterConstants.kV;
    flywheelConfig.Slot0.kP = ShooterConstants.kP;

    flywheelConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    flywheelConfig.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;

    flywheelConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    TalonFXUtil.applyConfigWithRetries(this.m_leftFlywheel, flywheelConfig, 5);
    TalonFXUtil.applyConfigWithRetries(this.m_rightFlywheel, flywheelConfig, 5);

    // Hood Settings
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.Slot0.kP = 75;

    hoodConfig.Feedback.SensorToMechanismRatio = 13.2;

    TalonFXUtil.applyConfigWithRetries(this.m_leftHood, hoodConfig, 5);
    TalonFXUtil.applyConfigWithRetries(this.m_rightHood, hoodConfig, 5);

    this.m_leftVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    this.m_rightVelocityRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
    this.m_leftPositionRequest = new PositionVoltage(0);
    this.m_rightPositionRequest = new PositionVoltage(0);

    m_leftFlywheelSignals = TalonFXUtil.getBasicStatusSignals(m_leftFlywheel);
    m_rightFlywheelSignals = TalonFXUtil.getBasicStatusSignals(m_rightFlywheel);
    m_leftHoodSignals = TalonFXUtil.getBasicStatusSignals(m_leftHood);
    m_rightHoodSignals = TalonFXUtil.getBasicStatusSignals(m_rightHood);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftFlywheelSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_rightFlywheelSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_leftHoodSignals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_rightHoodSignals);

    m_leftFlywheel.optimizeBusUtilization();
    m_rightFlywheel.optimizeBusUtilization();
    m_leftHood.optimizeBusUtilization();
    m_rightHood.optimizeBusUtilization();

    m_leftHood.setPosition(Degrees.of(ShooterConstants.HOOD_MIN));
    m_rightHood.setPosition(Degrees.of(ShooterConstants.HOOD_MIN));
  }

  public void update(double dtSeconds) {
    BaseStatusSignal.refreshAll(m_leftFlywheelSignals);
    BaseStatusSignal.refreshAll(m_rightFlywheelSignals);
    BaseStatusSignal.refreshAll(m_leftHoodSignals);
    BaseStatusSignal.refreshAll(m_rightHoodSignals);
  }

  public void setLeftTargetVelocity(AngularVelocity pVelocity) {
    if (pVelocity.in(RotationsPerSecond) == 0) {
      this.m_leftFlywheel.stopMotor();
    } else {
      this.m_leftFlywheel.setControl(this.m_leftVelocityRequest.withVelocity(pVelocity));
    }
  }

  public void setRightTargetVelocity(AngularVelocity pVelocity) {
    if (pVelocity.in(RotationsPerSecond) == 0) {
      this.m_leftFlywheel.stopMotor();
    } else {
      this.m_leftFlywheel.setControl(this.m_rightVelocityRequest.withVelocity(pVelocity));
    }
  }

  @NotLogged
  public AngularVelocity getLeftVelocity() {
    return this.m_leftFlywheel.getVelocity().getValue();
  }

  @NotLogged
  public AngularVelocity getRightVelocity() {
    return this.m_rightFlywheel.getVelocity().getValue();
  }

  @NotLogged
  public AngularVelocity getLeftTargetVelocity() {
    return RotationsPerSecond.of(this.m_leftVelocityRequest.Velocity);
  }

  @NotLogged
  public AngularVelocity getRightTargetVelocity() {
    return RotationsPerSecond.of(this.m_rightVelocityRequest.Velocity);
  }

  public void setLeftHoodTarget(Angle pAngle) {
    this.m_leftHood.setControl(
        m_leftPositionRequest.withPosition(
            Degrees.of(
                MathUtil.clamp(
                    pAngle.in(Degrees), ShooterConstants.HOOD_MIN, ShooterConstants.HOOD_MAX))));
  }

  public void setRightHoodTarget(Angle pAngle) {
    this.m_leftHood.setControl(
        m_leftPositionRequest.withPosition(
            Degrees.of(
                MathUtil.clamp(
                    pAngle.in(Degrees), ShooterConstants.HOOD_MIN, ShooterConstants.HOOD_MAX))));
  }

  @NotLogged
  public Angle getLeftHoodPosition() {
    return this.m_leftHood.getPosition().getValue();
  }

  @NotLogged
  public Angle getRightHoodPosition() {
    return this.m_leftHood.getPosition().getValue();
  }

  @NotLogged
  public boolean hasBreakerLeftBroke() {
    return this.m_leftBreamBreaker.get();
  }

  @NotLogged
  public boolean hasBreakerRightBroke() {
    return this.m_rightBreamBreaker.get();
  }

  public void setFuelSim(FuelSim fuelSim) {
    // CTRE implementation does not use FuelSim
  }
}
