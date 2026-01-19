package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.TalonFXUtil;
import frc.robot.Robot;

@Logged
public class ShooterIOCTRE implements ShooterIO {

  private final MotionMagicVelocityVoltage m_velocityManager;

  private final TalonFX m_flyWheel;
  private final TalonFX m_hood;

  public ShooterIOCTRE() {
    this.m_flyWheel = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, TunerConstants.kCANBus);
    this.m_hood = new TalonFX(ShooterConstants.HOOD_MOTOR_ID, TunerConstants.kCANBus);

    // Flywheel settings
    TalonFXConfiguration m_config = new TalonFXConfiguration();
    
    m_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_config.Slot0.kS = ShooterConstants.kS; // Static friction
    m_config.Slot0.kV = ShooterConstants.kV; // Velocity feedforward
    m_config.Slot0.kP = ShooterConstants.kP; // Proportional gain

    m_config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    m_config.MotionMagic.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;

    if (TalonFXUtil.applyConfigWithRetries(this.m_flyWheel, m_config, 2)) {
      Robot.telemetry().log("Flywheel/Config", true);
    } else {
      Robot.telemetry().log("Flywheel/Config", false);
    }

    
    this.m_velocityManager = new MotionMagicVelocityVoltage(0);
  }


  public void setVelocity(AngularVelocity velocity) {
    this.m_flyWheel.setControl(this.m_velocityManager.withVelocity(velocity));
  }

  public void stop() {
    this.m_flyWheel.stopMotor();
  }


  public void setHoodAngle(Angle angle) {
    this.m_hood.setPosition(angle);
  }

  public Angle getHoodAngle() {
    return this.m_hood.getPosition().getValue();
  }
}