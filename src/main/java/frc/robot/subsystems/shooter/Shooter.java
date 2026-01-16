package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;

@Logged
public class Shooter extends SubsystemBase {

  public enum SHOOTER_STATE {
    STOP(RotationsPerSecond.of(0)),

    IDLE(RotationsPerSecond.of(200)),
    SHOOT(RotationsPerSecond.of(250)),

    PRESET_PASS(RotationsPerSecond.of(100)),
    PRESET_SHOOT(RotationsPerSecond.of(250));


    private final AngularVelocity velocity;

    SHOOTER_STATE(AngularVelocity velocity) {
      this.velocity = velocity;
    }
  };

  private SHOOTER_STATE m_state;

  // Controller for spinning the Shooter at a target speed
  private final MotionMagicVelocityVoltage m_velocityManager;

  private final AngularVelocity m_tolerance;


  protected final TalonFX m_flyWheen;
  protected TalonFXConfiguration m_config;


  public Shooter() {
    this.m_state = SHOOTER_STATE.STOP;

    this.m_flyWheen = new TalonFX(21, TunerConstants.kCANBus);
    this.m_velocityManager = new MotionMagicVelocityVoltage(0);
    this.m_tolerance = RotationsPerSecond.of(ShooterConstants.VELOCITY_TOLERANCE_RPS);

    this.m_config = new TalonFXConfiguration();


    // Coast mode: Shooter can spin freely by hand when disabled
    this.m_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Set motor direction: positive power = counterclockwise spin
    this.m_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Control values from ShooterConstants
    this.m_config.Slot0.kS = ShooterConstants.kS; // Static friction
    this.m_config.Slot0.kV = ShooterConstants.kV; // Velocity feedforward
    this.m_config.Slot0.kP = ShooterConstants.kP; // Proportional gain

    this.m_config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    this.m_config.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;

    // Apply configuration with retries
    if (TalonFXUtil.applyConfigWithRetries(this.m_flyWheen, this.m_config, 2)) {
      Robot.telemetry().log("Shooter/Config", true);
    } else {
      Robot.telemetry().log("Shooter/Config", false);
    }
  }

  @Override
  public void periodic() {
    switch (this.m_state) {
      case STOP:
        this.m_flyWheen.stopMotor();
        
        break;
      

      case IDLE:
        setVelocity(SHOOTER_STATE.IDLE.velocity);
        break;

      case SHOOT:
        setVelocity(SHOOTER_STATE.SHOOT.velocity);
        break;


        case PRESET_PASS:
        setVelocity(SHOOTER_STATE.PRESET_PASS.velocity);
        break;
        
      case PRESET_SHOOT:
        setVelocity(SHOOTER_STATE.PRESET_SHOOT.velocity);
        break;
    }
  }


  public void setState(SHOOTER_STATE state) {
    this.m_state = state;
  }

  public SHOOTER_STATE getState() {
    return this.m_state;
  }


  private void setVelocity(AngularVelocity velocity) {
    this.m_flyWheen.setControl(this.m_velocityManager.withVelocity(velocity));
  }

  public AngularVelocity getVelocity() {
    return this.m_flyWheen.getVelocity().getValue();
  }


  public AngularVelocity getTargetVelocity() {
    return this.m_velocityManager.getVelocityMeasure();
  }


  public boolean isAtTarget() {
    return getVelocity().isNear(getTargetVelocity(), this.m_tolerance);
  }


  /**
   * Get the speed "Gace Zone" for the isAtTarget check.
   *
   * @return Speed tolerance
   */
  public AngularVelocity getTolerance() {
    return this.m_tolerance;
  }
}