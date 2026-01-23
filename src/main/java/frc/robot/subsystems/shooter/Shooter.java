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

  private AngularVelocity m_velocity;
  private Angle m_hoodAngle;

  public Shooter(ShooterIO io) {

    this.m_io = io;
    this.m_state = SHOOTER_STATE.STOP;
  }

  @Override
  public void periodic() {
    switch (this.m_state) {
      case SHOOT:
        this.m_io.setTargetVelocity(this.getTargetVelocity());
        this.m_io.setHoodAngle(this.getHoodAngle());
        break;

      default:
        this.m_io.setTargetVelocity(this.m_state.getVelocity());
        this.m_io.setHoodAngle(this.m_state.getHoodAngle());
        break;
    }

    

    // Stop motor if velocity is 0
    if (this.getTargetVelocity().in(DegreesPerSecond) == 0) {
      this.m_io.stop();
    }

    this.m_io.update();
  }

  public void setVelocity(AngularVelocity velocity) {
    this.m_velocity = velocity;
  }

  public AngularVelocity getVelocity() {
    return this.m_io.getVelocity();
  }

  public AngularVelocity getTargetVelocity() {
    return this.m_velocity;
  }

  public void setHoodAngle(Angle angle) {
    this.m_hoodAngle = angle;
  }

  public Angle getHoodAngle() {
    return this.m_hoodAngle;
  }

  public void setState(SHOOTER_STATE state) {
    this.m_state = state;
  }

  public SHOOTER_STATE getState() {
    return this.m_state;
  }
}
