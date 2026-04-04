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

/** CTRE hardware implementation of shooter IO. */
@Logged
public class ShooterIOCTRE implements ShooterIO {

  @NotLogged private final VelocityTorqueCurrentFOC m_velocityRequest;
  @NotLogged private final PositionVoltage m_hoodPositionRequest;
  // Four flywheel motors: left master, left follower, right master, right follower
  @NotLogged private final TalonFX[] m_flywheels;

  @Logged(importance = Importance.CRITICAL)
  private final TalonFX m_hoodMotor;

  @NotLogged private final BaseStatusSignal[] m_hoodSignals;

  @NotLogged private final BaseStatusSignal[][] m_flywheelSignals;

  @Logged(importance = Importance.INFO)
  private final DigitalInput m_leftBreamBreaker;

  @Logged(importance = Importance.INFO)
  private final DigitalInput m_rightBreamBreaker;

  /** Constructs the CTRE-backed shooter IO implementation. */
  public ShooterIOCTRE() {
    // Initialize flywheel motors (left master, left follower, right master, right follower)
    this.m_flywheels =
        new TalonFX[] {
          new TalonFX(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR),
          new TalonFX(CanIdConstants.Shooter.SHOOTER_LEFT_FOLLOWER),
          new TalonFX(CanIdConstants.Shooter.SHOOTER_RIGHT_MOTOR),
          new TalonFX(CanIdConstants.Shooter.SHOOTER_RIGHT_FOLLOWER),
        };

    this.m_hoodMotor = new TalonFX(CanIdConstants.Shooter.HOOD_MOTOR);

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

    // Apply flywheel config to all flywheel motors
    for (TalonFX fx : m_flywheels) {
      TalonFXUtil.applyConfigWithRetries(fx, flywheelConfig, 5);
    }

    // Hood Settings
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.Slot0.kP = 400;
    hoodConfig.Slot0.kD = 4;
    hoodConfig.Slot0.kG = 0.4;
    hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    hoodConfig.Feedback.SensorToMechanismRatio = 26.666666;

    TalonFXUtil.applyConfigWithRetries(this.m_hoodMotor, hoodConfig, 5);

    this.m_velocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);
    this.m_hoodPositionRequest = new PositionVoltage(0).withEnableFOC(true);

    // Collect status signals for each flywheel motor
    m_flywheelSignals = new BaseStatusSignal[m_flywheels.length][];
    for (int i = 0; i < m_flywheels.length; ++i) {
      m_flywheelSignals[i] = TalonFXUtil.getBasicStatusSignals(m_flywheels[i]);
    }
    m_hoodSignals = TalonFXUtil.getBasicStatusSignals(m_hoodMotor);

    for (BaseStatusSignal[] sigs : m_flywheelSignals) {
      BaseStatusSignal.setUpdateFrequencyForAll(50, sigs);
    }
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_hoodSignals);

    for (TalonFX fx : m_flywheels) {
      fx.optimizeBusUtilization();
    }
    m_hoodMotor.optimizeBusUtilization();

    m_hoodMotor.setPosition(Degrees.of(ShooterConstants.HOOD_MIN));
  }

  /** {@inheritDoc} */
  @Override
  public void update(double dtSeconds) {
    for (BaseStatusSignal[] sigs : m_flywheelSignals) {
      BaseStatusSignal.refreshAll(sigs);
    }
    BaseStatusSignal.refreshAll(m_hoodSignals);
  }

  /** {@inheritDoc} */
  @Override
  public void setTargetVelocity(AngularVelocity pVelocity) {
    if (pVelocity.in(RotationsPerSecond) == 0) {
      for (TalonFX flywheel : this.m_flywheels) {
        flywheel.stopMotor();
      }
      return;
    }

    for (TalonFX flywheel : this.m_flywheels) {
      flywheel.setControl(this.m_velocityRequest.withVelocity(pVelocity));
    }
  }

  /** {@inheritDoc} */
  @NotLogged
  @Override
  public AngularVelocity getVelocity() {
    return this.m_flywheels[0].getVelocity().getValue();
  }

  /**
   * Returns the current commanded flywheel target velocity.
   *
   * @return flywheel target angular velocity
   */
  @NotLogged
  public AngularVelocity getTargetVelocity() {
    return RotationsPerSecond.of(this.m_velocityRequest.Velocity);
  }

  /** {@inheritDoc} */
  @NotLogged
  @Override
  public void setHoodTarget(Angle pAngle) {
    this.m_hoodMotor.setControl(
        m_hoodPositionRequest.withPosition(
            Degrees.of(
                MathUtil.clamp(
                    pAngle.in(Degrees), ShooterConstants.HOOD_MIN, ShooterConstants.HOOD_MAX))));
  }

  /** {@inheritDoc} */
  @NotLogged
  @Override
  public Angle getHoodPosition() {
    return this.m_hoodMotor.getPosition().getValue();
  }

  /** {@inheritDoc} */
  @NotLogged
  @Override
  public boolean hasBreakerLeftBroke() {
    return this.m_leftBreamBreaker.get();
  }

  /** {@inheritDoc} */
  @NotLogged
  @Override
  public boolean hasBreakerRightBroke() {
    return this.m_rightBreamBreaker.get();
  }

  /** {@inheritDoc} */
  @Override
  public void setFuelSim(FuelSim fuelSim) {
    // CTRE implementation does not use FuelSim
  }
}
