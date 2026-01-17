package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;

@Logged
public class ShooterIOCTRE implements ShooterIO {

  private final MotionMagicVelocityVoltage m_velocityManager;

  private final TalonFX m_flyWheen;
  private final TalonFX m_hood;

  public ShooterIOCTRE() {
    this.m_flyWheen = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, TunerConstants.kCANBus);
    this.m_hood = new TalonFX(ShooterConstants.HOOD_MOTOR_ID, TunerConstants.kCANBus);

    this.m_velocityManager = new MotionMagicVelocityVoltage(0);
  }


  public void setVelocity(AngularVelocity velocity) {
    this.m_flyWheen.setControl(this.m_velocityManager.withVelocity(velocity));
  }

  public void stop() {
    this.m_flyWheen.stopMotor();
  }


  public void setHoodAngle(Angle angle) {
    this.m_hood.setPosition(angle);
  }

  public Angle getHoodAngle() {
    return this.m_hood.getPosition().getValue();
  }
}