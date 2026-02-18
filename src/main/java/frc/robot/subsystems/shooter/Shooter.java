package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.filter.Debouncer;
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
  @Logged(importance = Importance.DEBUG)
  private SHOOTER_STATE m_currentState;

  @Logged(importance = Importance.DEBUG)
  private final ShooterIO m_io;

  @Logged(importance = Importance.DEBUG)
  private final TimesConsumer m_timesConsumer;

  @Logged(importance = Importance.DEBUG)
  private AngularVelocity m_velocity;

  @Logged(importance = Importance.DEBUG)
  private Angle m_hoodAngle;

  @Logged(importance = Importance.DEBUG)
  private boolean m_isGoingTowardsAllianceZone;

  @Logged(importance = Importance.DEBUG)
  private boolean m_didIntake;

  @Logged(importance = Importance.DEBUG)
  private Timer m_FPSTimer;

  @Logged(importance = Importance.DEBUG)
  private double m_FPS;

  @Logged(importance = Importance.DEBUG)
  private int m_fuelCount;

  @Logged(importance = Importance.DEBUG)
  private Debouncer m_jamDetect;

  public Shooter(ShooterIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_currentState = SHOOTER_STATE.STOP;

    this.m_velocity = RotationsPerSecond.of(0);
    this.m_hoodAngle = Degrees.of(0);

    this.m_isGoingTowardsAllianceZone = false;
    this.m_didIntake = false;

    this.m_fuelCount = 0;
    this.m_FPS = 0;

    this.m_FPSTimer = new Timer();
    this.m_FPSTimer.start();

    this.m_jamDetect = new Debouncer(ShooterConstants.JAM_DETECT_TIME);
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

    if (this.m_io.hasBreakerBroke() || this.m_io.hasBreakerFollowerBroke()) {
      if (this.m_FPSTimer.get() != 0) {
        this.m_FPS = 1 / (2 * this.m_FPSTimer.get());
      }
      this.m_FPSTimer.reset();

      this.m_fuelCount++;
    }

    this.m_io.setTargetVelocity(this.m_velocity);
    this.m_io.setHoodAngle(this.m_hoodAngle);

    this.m_io.update(this.m_currentState.m_subsystemPeriodicFrequency.in(Seconds));
  }

  @Logged(importance = Importance.DEBUG)
  public AngularVelocity getVelocity() {
    return this.m_io.getVelocity();
  }

  @Logged(importance = Importance.DEBUG)
  public AngularVelocity getTargetVelocity() {
    return this.m_velocity;
  }

  @Logged(importance = Importance.DEBUG)
  public boolean isAtTargetVelocity() {
    return this.getVelocity()
        .isNear(getTargetVelocity(), ShooterConstants.FLYWHEEL_TARGET_ERROR_RANGE);
  }

  @Logged(importance = Importance.DEBUG)
  public boolean isHoodAtTargetAngle() {
    return this.getHoodAngle()
        .isNear(getTargetHoodAngle(), ShooterConstants.HOOD_TARGET_ERROR_RANGE);
  }

  @Logged(importance = Importance.DEBUG)
  public Angle getTargetHoodAngle() {
    return this.m_hoodAngle;
  }

  @Logged(importance = Importance.DEBUG)
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
    IDLE(Milliseconds.of(20), RotationsPerSecond.of(0), Degrees.of(0)),
    SHOOT(Milliseconds.of(20), RotationsPerSecond.of(60), Degrees.of(0)),
    PRESET_SCORE(Milliseconds.of(60), RotationsPerSecond.of(60), Degrees.of(0));

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
    // if (pState == SHOOTER_STATE.IDLE && m_isGoingTowardsAllianceZone && m_didIntake) {
    //   this.m_state = SHOOTER_STATE.PRESET_SCORE;
    // } else {
    this.m_currentState = pState;
    // }
  }

  @Logged(importance = Importance.DEBUG)
  public SHOOTER_STATE getState() {
    return this.m_currentState;
  }

  @Logged(importance = Importance.DEBUG)
  public int getBallCount() {
    return this.m_fuelCount;
  }

  @Logged(importance = Importance.DEBUG)
  public void resetBallCount() {
    this.m_fuelCount = 0;
  }

  @Logged(importance = Importance.DEBUG)
  public double getFPS() {
    return this.m_FPS;
  }

  @Logged(importance = Importance.DEBUG)
  public boolean isJammed() {
    return this.m_jamDetect.calculate(
        !this.m_io.hasBreakerBroke() && !this.m_io.hasBreakerFollowerBroke());
  }

  @Logged(importance = Importance.DEBUG)
  public boolean hasBreakerBroke() {
    return this.m_io.hasBreakerBroke();
  }

  @Logged(importance = Importance.DEBUG)
  public boolean hasBreakerFollowerBroke() {
    return this.m_io.hasBreakerFollowerBroke();
  }
}
