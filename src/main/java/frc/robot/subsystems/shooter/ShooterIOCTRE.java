package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.FuelSim;
import frc.robot.utils.TalonFXUtil;

/** CTRE hardware implementation of shooter IO. */
public class ShooterIOCTRE implements ShooterIO {

  private final MotionMagicVelocityTorqueCurrentFOC m_velocityRequest;
  private final MotionMagicVoltage m_hoodPositionRequest;

  // Four flywheel motors: left master, left follower, right follower, right follower
  private final TalonFX[] m_flywheels;

  private final TalonFX m_leftFlywheelLeader;

  private final TalonFX m_leftFlywheelFollower;

  private final TalonFX m_rightFlywheelFollower1;

  private final TalonFX m_rightFlywheelFollower2;

  private final TalonFX m_hoodMotor;

  private final BaseStatusSignal[] m_hoodSignals;

  private final BaseStatusSignal[] m_flywheelSignals;

  private final BaseStatusSignal m_flywheelSetpointVelocitySignal;

  /** Constructs the CTRE-backed shooter IO implementation. */
  public ShooterIOCTRE() {
    // Initialize flywheel motors:
    // left (master), left (follower: aligned), right (follower: opposed), right (follower: opposed)
    m_leftFlywheelLeader = new TalonFX(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR);
    m_leftFlywheelFollower = new TalonFX(CanIdConstants.Shooter.SHOOTER_LEFT_FOLLOWER);
    m_rightFlywheelFollower1 = new TalonFX(CanIdConstants.Shooter.SHOOTER_RIGHT_FOLLOWER_1);
    m_rightFlywheelFollower2 = new TalonFX(CanIdConstants.Shooter.SHOOTER_RIGHT_FOLLOWER_2);
    this.m_flywheels =
        new TalonFX[] {
          m_leftFlywheelLeader,
          m_leftFlywheelFollower,
          m_rightFlywheelFollower1,
          m_rightFlywheelFollower2
        };

    this.m_hoodMotor = new TalonFX(CanIdConstants.Shooter.HOOD_MOTOR);

    // ================ Flywheel settings ================ \\
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    flywheelConfig.Slot0.kS = ShooterConstants.kFlyS;
    flywheelConfig.Slot0.kV = ShooterConstants.kFlyV;
    flywheelConfig.Slot0.kA = ShooterConstants.kFlyA;
    flywheelConfig.Slot0.kP = ShooterConstants.kFlyP;
    // Only use kP when at setpoint so it can be tuned more aggressively

    flywheelConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.FLYWHEEL_MOTION_MAGIC_ACCELERATION;
    flywheelConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.FLYWHEEL_MOTION_MAGIC_CRUISE_VELOCITY;

    flywheelConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Apply flywheel config to all flywheel motors
    for (TalonFX fx : m_flywheels) {
      TalonFXUtil.applyConfigWithRetries(fx, flywheelConfig, 5);
    }

    // Configure follower motors to follow their respective masters
    m_flywheels[1].setControl(
        new Follower(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR, MotorAlignmentValue.Aligned)
            .withUpdateFreqHz(200));
    m_flywheels[2].setControl(
        new Follower(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR, MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(200));
    m_flywheels[3].setControl(
        new Follower(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR, MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(200));

    m_flywheelSetpointVelocitySignal = m_flywheels[0].getClosedLoopReferenceSlope();
    m_flywheelSetpointVelocitySignal.setUpdateFrequency(50);

    // Hood Settings
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodConfig.Slot0.kP = ShooterConstants.kHoodP;
    hoodConfig.Slot0.kS = ShooterConstants.kHoodS;
    hoodConfig.Slot0.kG = ShooterConstants.kHoodG;
    hoodConfig.Slot0.kV = ShooterConstants.kHoodV;
    hoodConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    hoodConfig.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.HOOD_MOTION_MAGIC_ACCELERATION;
    hoodConfig.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.HOOD_MOTION_MAGIC_CRUISE_VELOCITY;

    // 2 3:1 gearboxes with custom gear ratio at the end
    hoodConfig.Feedback.SensorToMechanismRatio = (5.0 / 1.0) * (5.0 / 1.0) * (70.0 / 11.0);

    TalonFXUtil.applyConfigWithRetries(this.m_hoodMotor, hoodConfig, 5);

    this.m_velocityRequest = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
    this.m_hoodPositionRequest = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);

    // Configure status signals for each motor
    m_hoodSignals = TalonFXUtil.getBasicStatusSignals(m_hoodMotor);
    BaseStatusSignal.setUpdateFrequencyForAll(50, m_hoodSignals);
    m_hoodMotor.optimizeBusUtilization();

    m_flywheelSignals = TalonFXUtil.getBasicStatusSignals(m_flywheels);
    for (int i = 0; i < m_flywheelSignals.length; ++i) {
      BaseStatusSignal.setUpdateFrequencyForAll(50, m_flywheelSignals[i]);
    }

    // Configure leader update frequency to optimize follower performance
    // https://www.chiefdelphi.com/t/ctre-follower-does-the-same-volts-or-the-same-control-request/513725/3?u=carterc13
    // m_flywheels[0].getMotorVoltage().setUpdateFrequency(200);
    m_flywheels[0].getTorqueCurrent().setUpdateFrequency(200);

    for (TalonFX fx : m_flywheels) {
      fx.optimizeBusUtilization();
    }

    m_hoodMotor.setPosition(Degrees.of(ShooterConstants.HOOD_MIN));
  }

  /** {@inheritDoc} */
  @Override
  public void update(double dtSeconds) {
    BaseStatusSignal.refreshAll(m_flywheelSignals);
    BaseStatusSignal.refreshAll(m_hoodSignals);
    BaseStatusSignal.refreshAll(m_flywheelSetpointVelocitySignal);
  }

  /** {@inheritDoc} */
  @Override
  public void setTargetVelocity(AngularVelocity pVelocity) {
    // Only leader motor needs to be commanded
    if (pVelocity.in(RotationsPerSecond) == 0) {
      m_flywheels[0].stopMotor();
      return;
    }

    m_flywheels[0].setControl(this.m_velocityRequest.withVelocity(pVelocity));
    // m_flywheels[1].setControl(this.m_velocityRequest.withVelocity(pVelocity));
    // m_flywheels[2].setControl(this.m_velocityRequest.withVelocity(pVelocity.times(-1)));
    // m_flywheels[3].setControl(this.m_velocityRequest.withVelocity(pVelocity.times(-1)));
  }

  /** Returns the closed loop reference slope == 0 */
  @Override
  public boolean getSetpointReferenceVelocityIsZero() {
    return Math.abs(m_flywheelSetpointVelocitySignal.getValueAsDouble()) <= 2.5;
  }

  /** {@inheritDoc} */
  @Override
  public AngularVelocity getVelocity() {
    var avg = 0.0;
    for (TalonFX fx : m_flywheels) {
      avg += fx.getVelocity(false).getValue().abs(RotationsPerSecond);
    }
    return RotationsPerSecond.of(avg / m_flywheels.length);
  }

  /** {@inheritDoc} */
  @Override
  public void setHoodTarget(Angle pAngle) {
    this.m_hoodMotor.setControl(
        m_hoodPositionRequest.withPosition(
            Degrees.of(
                MathUtil.clamp(
                    pAngle.in(Degrees), ShooterConstants.HOOD_MIN, ShooterConstants.HOOD_MAX))));
  }

  /** {@inheritDoc} */
  @Override
  public Angle getHoodPosition() {
    return this.m_hoodMotor.getPosition(false).getValue();
  }

  /** {@inheritDoc} */
  @Override
  public void setFuelSim(FuelSim fuelSim) {
    // CTRE implementation does not use FuelSim
  }
}
