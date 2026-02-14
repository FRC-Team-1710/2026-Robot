package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import frc.robot.utils.shooterMath.ShooterMath;

@Logged
public class Shooter {
  private SHOOTER_STATE m_currentState;

  private final ShooterIO m_io;

  private final TimesConsumer m_timesConsumer;

  private AngularVelocity m_velocity;
  private Angle m_hoodAngle;

  private boolean m_isGoingTowardsAllianceZone;
  private boolean m_didIntake;

  private int m_ballcount;

  private Timer m_ballTimer;
  private Timer m_ballFollowerTimer;

  private double m_ballsPerSec;
  private double m_followerBallsPerSec;

  public Shooter(ShooterIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_currentState = SHOOTER_STATE.STOP;

    this.m_velocity = RotationsPerSecond.of(0);
    this.m_hoodAngle = Degrees.of(0);

    this.m_isGoingTowardsAllianceZone = false;
    this.m_didIntake = false;

    this.m_ballcount = 0;

    this.m_ballTimer = new Timer();
    this.m_ballFollowerTimer = new Timer();

    this.m_ballsPerSec = 0.0;
    this.m_followerBallsPerSec = 0.0;
  }

  public void periodic() {

    switch (this.m_currentState) {
      case SHOOT:
        this.m_velocity = ShooterMath.getShooterRPM();
        this.m_hoodAngle = ShooterMath.getShooterAngle();
        break;

      default:
        this.m_velocity = this.m_currentState.m_velocity;
        this.m_hoodAngle = this.m_currentState.m_hoodAngle;
        break;
    }

    if (this.m_ballTimer.get() >= 1) {
      this.m_ballsPerSec = 0;
    }

    if (this.m_io.hasBreakerBroke()) {
      this.m_ballsPerSec = 1.0 / this.m_ballTimer.get();
      this.m_ballTimer.reset();
    }

    if (this.m_ballFollowerTimer.get() >= 1) {
      this.m_followerBallsPerSec = 0;
    }

    if (this.m_io.hasBreakerFollowerBroke()) {
      this.m_followerBallsPerSec = 1.0 / this.m_ballFollowerTimer.get();
      this.m_ballFollowerTimer.reset();
    }

    this.m_io.setTargetVelocity(this.m_velocity);
    this.m_io.setHoodAngle(this.m_hoodAngle);

    this.m_io.update(m_currentState.m_subsystemPeriodicFrequency.in(Seconds));
  }

  public AngularVelocity getVelocity() {
    return this.m_io.getVelocity();
  }

  public AngularVelocity getTargetVelocity() {
    return this.m_velocity;
  }

  public boolean isAtTargetVelocity() {
    return this.getVelocity()
        .isNear(getTargetVelocity(), ShooterConstants.FLYWHEEL_TARGET_ERROR_RANGE);
  }

  public boolean isHoodAtTargetAngle() {
    return this.getHoodAngle()
        .isNear(getTargetHoodAngle(), ShooterConstants.HOOD_TARGET_ERROR_RANGE);
  }

  public Angle getTargetHoodAngle() {
    return this.m_hoodAngle;
  }

  public Angle getHoodAngle() {
    return this.m_io.getHoodAngle();
  }

  public void setGoingTowardsAllianceZone(boolean isGoingTowardsAllianceZone) {
    this.m_isGoingTowardsAllianceZone = isGoingTowardsAllianceZone;
  }

  public void setDidIntake(boolean didIntake) {
    this.m_didIntake = didIntake;
  }

  public enum SHOOTER_STATE {
    STOP(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(0)),
    IDLE(Milliseconds.of(60), RotationsPerSecond.of(250), Degrees.of(0)),
    SHOOT(Milliseconds.of(20), RotationsPerSecond.of(0), Degrees.of(0)),
    PRESET_SCORE(Milliseconds.of(60), RotationsPerSecond.of(750), Degrees.of(0));

    private final Time m_subsystemPeriodicFrequency;
    private final AngularVelocity m_velocity;
    private final Angle m_hoodAngle;

    SHOOTER_STATE(Time subsystemPeriodicFrequency, AngularVelocity velocity, Angle hoodAngle) {
      this.m_subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.m_velocity = velocity;
      this.m_hoodAngle = hoodAngle;
    }
  }

  public void setState(SHOOTER_STATE pState) {
    if (!this.m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        pState.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Shooter, pState.m_subsystemPeriodicFrequency);
    }
    if (pState == SHOOTER_STATE.IDLE && m_isGoingTowardsAllianceZone && m_didIntake) {
      this.m_currentState = SHOOTER_STATE.PRESET_SCORE;
    } else {
      this.m_currentState = pState;
    }
  }

  public SHOOTER_STATE getState() {
    return this.m_currentState;
  }

  public int getBallCount() {
    return this.m_ballcount;
  }

  public void resetBallCount() {
    this.m_ballcount = 0;
  }

  public double getBallsPerSecond() {
    if (this.m_ballsPerSec == 0) {
      return this.m_followerBallsPerSec;
    } else if (this.m_followerBallsPerSec == 0) {
      return this.m_ballsPerSec;
    } else {
      return (this.m_ballsPerSec + this.m_followerBallsPerSec) / 2;
    }
  }
}
