package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Shooter {
  private SHOOTER_STATE m_state;

  private final ShooterIO m_io;

  private final TimesConsumer m_timesConsumer;

  private AngularVelocity m_velocity = RotationsPerSecond.of(0);
  private Angle m_hoodAngle = Degrees.of(0);

  public Shooter(ShooterIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_state = SHOOTER_STATE.STOP;
  }

  public void periodic() {
    switch (this.m_state) {
      case SHOOT:
        this.m_io.setTargetVelocity(this.getTargetVelocity());
        this.m_io.setHoodAngle(this.getTargetHoodAngle());
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

  public void setVelocity(AngularVelocity pVelocity) {
    this.m_velocity = pVelocity;
  }

  public AngularVelocity getVelocity() {
    return this.m_io.getVelocity();
  }

  public AngularVelocity getTargetVelocity() {
    return this.m_velocity;
  }

  public boolean isAtTargetVelocity() {
    return this.getVelocity().isNear(getTargetVelocity(), ShooterConstants.TARGET_ERROR_RANGE);
  }

  public void setTargetHoodAngle(Angle pAngle) {
    this.m_hoodAngle = pAngle;
  }

  public Angle getTargetHoodAngle() {
    return this.m_hoodAngle;
  }

  public enum SHOOTER_STATE {
    STOP(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(0)),
    IDLE(Milliseconds.of(60), RotationsPerSecond.of(200), Degrees.of(0)),
    SHOOT(Milliseconds.of(20), RotationsPerSecond.of(250), Degrees.of(0)),
    PRESET_PASS(Milliseconds.of(20), RotationsPerSecond.of(100), Degrees.of(0)),
    PRESET_SHOOT(Milliseconds.of(20), RotationsPerSecond.of(250), Degrees.of(0));

    private final Time m_subsystemPeriodicFrequency;
    private final AngularVelocity m_velocity;
    private final Angle m_hoodAngle;

    SHOOTER_STATE(Time subsystemPeriodicFrequency, AngularVelocity velocity, Angle hoodAngle) {
      this.m_subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.m_velocity = velocity;
      this.m_hoodAngle = hoodAngle;
    }

    Time getSubsystemPeriodicFrequency() {
      return this.m_subsystemPeriodicFrequency;
    }

    AngularVelocity getVelocity() {
      return this.m_velocity;
    }

    Angle getHoodAngle() {
      return this.m_hoodAngle;
    }
  }

  public void setState(SHOOTER_STATE pState) {
    if (!this.m_state
        .getSubsystemPeriodicFrequency()
        .isEquivalent(pState.getSubsystemPeriodicFrequency())) {
      m_timesConsumer.accept(Subsystems.Shooter, pState.getSubsystemPeriodicFrequency());
    }
    this.m_state = pState;
  }

  public SHOOTER_STATE getState() {
    return this.m_state;
  }
}
