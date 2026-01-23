package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Shooter extends SubsystemBase {

  public enum SHOOTER_STATE {
    STOP(RotationsPerSecond.of(0), Degrees.of(0)),

    IDLE(RotationsPerSecond.of(200), Degrees.of(0)),
    SHOOT(RotationsPerSecond.of(250), Degrees.of(0)),

    PRESET_PASS(RotationsPerSecond.of(100), Degrees.of(0)),
    PRESET_SHOOT(RotationsPerSecond.of(250), Degrees.of(0));

    private final AngularVelocity m_velocity;
    private final Angle m_hoodAngle;

    SHOOTER_STATE(AngularVelocity velocity, Angle hoodAngle) {
      this.m_velocity = velocity;
      this.m_hoodAngle = hoodAngle;
    }

    AngularVelocity getVelocity() {
      return this.m_velocity;
    }

    Angle getHoodAngle() {
      return this.m_hoodAngle;
    }
  };

  private SHOOTER_STATE m_state;

  private final ShooterIO m_io;

  public Shooter(ShooterIO io) {

    this.m_io = io;
    this.m_state = SHOOTER_STATE.STOP;
  }

  @Override
  public void periodic() {
    AngularVelocity velocity;
    Angle hoodAngle;

    switch (this.m_state) {
      case SHOOT:
        velocity = this.m_io.getTargetVelocity();
        hoodAngle = this.m_io.getHoodAngle();
        break;

      default:
        velocity = this.m_state.getVelocity();
        hoodAngle = this.m_state.getHoodAngle();
        break;
    }

    this.m_io.setTargetVelocity(velocity);
    this.m_io.setHoodAngle(hoodAngle);

    // Stop motor if velocity is 0
    if (velocity.in(DegreesPerSecond) == 0) {
      this.m_io.stop();
    }

    this.m_io.update();
  }

  public void setVelocity(AngularVelocity velocity) {
    this.m_io.setTargetVelocity(velocity);
  }

  public AngularVelocity getVelocity() {
    return this.m_io.getTargetVelocity();
  }

  public void setHoodAngle(Angle angle) {
    this.m_io.setHoodAngle(angle);
  }

  public Angle getHoodAngle() {
    return this.m_io.getHoodAngle();
  }

  public void setState(SHOOTER_STATE state) {
    this.m_state = state;
  }

  public SHOOTER_STATE getState() {
    return this.m_state;
  }
}
