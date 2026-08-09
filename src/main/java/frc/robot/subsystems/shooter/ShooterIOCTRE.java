package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Robot;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.SubsystemConstants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.FuelSim;
import frc.robot.utils.TalonFXUtil;

@Logged
public class ShooterIOCTRE implements ShooterIO {

  private final VelocityVoltage m_velocityVoltage =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

  private final MotionMagicVoltage m_mmPosition =
      new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);

  private final TalonFX m_leftLeader;
  private final TalonFX m_leftFollower;
  private final TalonFX m_rightFollower;
  private final TalonFX m_rightFollower2;
  private final TalonFX m_hoodMotor;

  public ShooterIOCTRE() {
    m_hoodMotor = new TalonFX(CanIdConstants.Shooter.HOOD_MOTOR);
    m_leftLeader = new TalonFX(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR, TunerConstants.kCANBus);
    m_leftFollower =
        new TalonFX(CanIdConstants.Shooter.SHOOTER_LEFT_FOLLOWER, TunerConstants.kCANBus);
    m_rightFollower =
        new TalonFX(CanIdConstants.Shooter.SHOOTER_RIGHT_FOLLOWER, TunerConstants.kCANBus);
    m_rightFollower2 =
        new TalonFX(CanIdConstants.Shooter.SHOOTER_RIGHT_FOLLOWER_2, TunerConstants.kCANBus);

    TalonFXUtil.applyConfig(ShooterConstants.Hood.Software.Config.kConfig, m_hoodMotor);
    TalonFXUtil.applyConfig(
        ShooterConstants.Flywheel.Software.Config.kConfig,
        m_leftLeader,
        m_leftFollower,
        m_rightFollower,
        m_rightFollower2);

    // https://www.chiefdelphi.com/t/ctre-follower-does-the-same-volts-or-the-same-control-request/513725/3?u=carterc13
    m_leftLeader
        .getMotorVoltage()
        .setUpdateFrequency(ShooterConstants.Flywheel.Software.kFollowerUpdateFrequency);

    TalonFXUtil.optimizeForPIDStatusSignalsExcludingVoltage(m_leftLeader);
    TalonFXUtil.optimizeForBasicStatusSignals(m_hoodMotor);

    m_leftFollower.optimizeBusUtilization();
    m_rightFollower.optimizeBusUtilization();
    m_rightFollower2.optimizeBusUtilization();

    m_hoodMotor.setPosition(ShooterConstants.Hood.Hardware.kHoodMin);

    // Configure follower motors to follow their master
    m_leftFollower.setControl(
        new Follower(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR, MotorAlignmentValue.Aligned)
            .withUpdateFreqHz(ShooterConstants.Flywheel.Software.kFollowerUpdateFrequency));
    m_rightFollower.setControl(
        new Follower(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR, MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(ShooterConstants.Flywheel.Software.kFollowerUpdateFrequency));
    m_rightFollower2.setControl(
        new Follower(CanIdConstants.Shooter.SHOOTER_LEFT_MOTOR, MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(ShooterConstants.Flywheel.Software.kFollowerUpdateFrequency));
  }

  /** {@inheritDoc} */
  @Override
  public void setTargetVelocity(AngularVelocity velocity) {
    // Only leader motor needs to be commanded
    if (velocity.in(RotationsPerSecond) == 0) {
      m_leftLeader.stopMotor();
      return;
    }

    m_leftLeader.setControl(m_velocityVoltage.withVelocity(velocity));
  }

  /** {@inheritDoc} */
  @Override
  public void setHoodTarget(Angle angle) {
    m_hoodMotor.setControl(
        m_mmPosition.withPosition(
            Degrees.of(
                MathUtil.clamp(
                    angle.in(Degrees),
                    ShooterConstants.Hood.Hardware.kHoodMin.in(Degrees),
                    ShooterConstants.Hood.Hardware.kHoodMax.in(Degrees)))));
  }

  /** {@inheritDoc} */
  @Override
  public Angle getHoodPosition() {
    return m_hoodMotor.getPosition().getValue();
  }

  /** {@inheritDoc} */
  @Override
  public void dynamicCurrentLimit(Current supply, Current stator) {
    Robot.telemetry().log("ShooterDynamicCurrentLimit/Supply", supply);
    Robot.telemetry().log("ShooterDynamicCurrentLimit/Stator", stator);
    var newConfig =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(stator)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(supply)
            .withSupplyCurrentLimitEnable(true);

    m_leftLeader.getConfigurator().apply(newConfig, 0.0);
    m_leftFollower.getConfigurator().apply(newConfig, 0.0);
    m_rightFollower.getConfigurator().apply(newConfig, 0.0);
    m_rightFollower2.getConfigurator().apply(newConfig, 0.0);
  }

  /** {@inheritDoc} */
  @Override
  public void setFuelSim(FuelSim fuelSim) {}
}
