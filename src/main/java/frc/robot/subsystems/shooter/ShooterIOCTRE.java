package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

  @NotLogged private final VelocityTorqueCurrentFOC m_VelocityRequest1;
  @NotLogged private final VelocityTorqueCurrentFOC m_VelocityRequest2;
  @NotLogged private final VelocityTorqueCurrentFOC m_VelocityRequest3;
  @NotLogged private final VelocityTorqueCurrentFOC m_VelocityRequest4;

  @NotLogged private final PositionVoltage m_hoodPositionRequest;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_flywheel1;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_flywheel2;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_flywheel3;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_flywheel4;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_hood;

  @NotLogged private final BaseStatusSignal[] m_flywheel1Signals;
  @NotLogged private final BaseStatusSignal[] m_flywheel2Signals;
  @NotLogged private final BaseStatusSignal[] m_flywheel3Signals;
  @NotLogged private final BaseStatusSignal[] m_flywheel4Signals;
  @NotLogged private final BaseStatusSignal[] m_hoodSignals;

  @Logged(importance = Importance.INFO)
  private final DigitalInput m_leftBreamBreaker;

  @Logged(importance = Importance.INFO)
  private final DigitalInput m_rightBreamBreaker;

  public ShooterIOCTRE() {
    this.m_flywheel1 = new TalonFX(CanIdConstants.Shooter.SHOOTER_FLYWHEEL_1);
    this.m_flywheel2 = new TalonFX(CanIdConstants.Shooter.SHOOTER_FLYWHEEL_2);
    this.m_flywheel3 = new TalonFX(CanIdConstants.Shooter.SHOOTER_FLYWHEEL_3);
    this.m_flywheel4 = new TalonFX(CanIdConstants.Shooter.SHOOTER_FLYWHEEL_4);

    this.m_hood = new TalonFX(CanIdConstants.Shooter.HOOD_MOTOR);

    // TODO: Do we need 2 beam breakers
    this.m_leftBreamBreaker = new DigitalInput(8);
    this.m_rightBreamBreaker = new DigitalInput(9);

    // Flywheel settings
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    flywheelConfig.Slot0.kS = ShooterConstants.kS;
    flywheelConfig.Slot0.kV = ShooterConstants.kV;
    flywheelConfig.Slot0.kP = ShooterConstants.kP;

    flywheelConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -10;

    TalonFXUtil.applyConfigWithRetries(this.m_flywheel1, flywheelConfig, 5);
    TalonFXUtil.applyConfigWithRetries(this.m_flywheel2, flywheelConfig, 5);
    TalonFXUtil.applyConfigWithRetries(this.m_flywheel3, flywheelConfig, 5);
    TalonFXUtil.applyConfigWithRetries(this.m_flywheel4, flywheelConfig, 5);

    // Hood Settings
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.Slot0.kP = 400;
    hoodConfig.Slot0.kD = 4;
    hoodConfig.Slot0.kG = 0.4;
    hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    hoodConfig.Feedback.SensorToMechanismRatio = 26.666666;

    TalonFXUtil.applyConfigWithRetries(this.m_hood, hoodConfig, 5);

    this.m_VelocityRequest1 = new VelocityTorqueCurrentFOC(0).withSlot(0);
    this.m_VelocityRequest2 = new VelocityTorqueCurrentFOC(0).withSlot(0);
    this.m_VelocityRequest3 = new VelocityTorqueCurrentFOC(0).withSlot(0);
    this.m_VelocityRequest4 = new VelocityTorqueCurrentFOC(0).withSlot(0);
    this.m_hoodPositionRequest = new PositionVoltage(0).withEnableFOC(true);

    m_flywheel1Signals = TalonFXUtil.getBasicStatusSignals(m_flywheel1);
    m_flywheel2Signals = TalonFXUtil.getBasicStatusSignals(m_flywheel2);
    m_flywheel3Signals = TalonFXUtil.getBasicStatusSignals(m_flywheel3);
    m_flywheel4Signals = TalonFXUtil.getBasicStatusSignals(m_flywheel4);
    m_hoodSignals = TalonFXUtil.getBasicStatusSignals(m_hood);

    BaseStatusSignal.setUpdateFrequencyForAll(50, m_flywheel1Signals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_flywheel2Signals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_flywheel3Signals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_flywheel4Signals);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_hoodSignals);

    m_flywheel1.optimizeBusUtilization();
    m_flywheel2.optimizeBusUtilization();
    m_hood.optimizeBusUtilization();

    m_hood.setPosition(Degrees.of(ShooterConstants.HOOD_MIN));
  }

  public void update(double dtSeconds) {
    BaseStatusSignal.refreshAll(m_flywheel1Signals);
    BaseStatusSignal.refreshAll(m_flywheel2Signals);
    BaseStatusSignal.refreshAll(m_hoodSignals);
    BaseStatusSignal.refreshAll(m_hoodSignals);
  }

  public void setTargetVelocity(AngularVelocity pVelocity) {
    if (pVelocity.in(RotationsPerSecond) == 0) {
      this.m_flywheel1.stopMotor();
      this.m_flywheel2.stopMotor();
      this.m_flywheel3.stopMotor();
      this.m_flywheel4.stopMotor();
    } else {
      this.m_flywheel1.setControl(this.m_VelocityRequest1.withVelocity(pVelocity));
      this.m_flywheel2.setControl(this.m_VelocityRequest2.withVelocity(pVelocity));
      this.m_flywheel3.setControl(this.m_VelocityRequest3.withVelocity(pVelocity));
      this.m_flywheel4.setControl(this.m_VelocityRequest4.withVelocity(pVelocity));
    }
  }

  @NotLogged
  public AngularVelocity getAverageVelocity() {
    return this.m_flywheel1
        .getVelocity()
        .getValue()
        .plus(m_flywheel2.getVelocity().getValue())
        .plus(m_flywheel3.getVelocity().getValue())
        .plus(m_flywheel3.getVelocity().getValue())
        .div(4);
  }

  public void setHoodTarget(Angle pAngle) {
    this.m_hood.setControl(
        m_hoodPositionRequest.withPosition(
            Degrees.of(
                MathUtil.clamp(
                    pAngle.in(Degrees), ShooterConstants.HOOD_MIN, ShooterConstants.HOOD_MAX))));
  }

  @NotLogged
  public Angle getHoodPosition() {
    return this.m_hood.getPosition().getValue();
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
