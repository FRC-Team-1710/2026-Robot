package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import frc.robot.utils.FuelSim;
import frc.robot.utils.shooterMath.ShooterMath;
import java.util.ArrayList;

@Logged
public class Shooter {
  @Logged(importance = Importance.CRITICAL)
  private SHOOTER_STATE m_currentState;

  @Logged(importance = Importance.CRITICAL)
  private final ShooterIO m_io;

  @NotLogged private final TimesConsumer m_timesConsumer;

  @Logged(importance = Importance.CRITICAL)
  private AngularVelocity m_velocity;

  @Logged(importance = Importance.CRITICAL)
  private Angle m_hoodAngle;

  @NotLogged private boolean m_isGoingTowardsAllianceZone;

  @Logged(importance = Importance.INFO)
  private boolean m_didIntake;

  @NotLogged private Timer m_FPSTimer;

  @Logged(importance = Importance.INFO)
  private double m_FPS;

  @NotLogged private ArrayList<Double> m_FPSLists;

  @Logged(importance = Importance.CRITICAL)
  private int m_fuelCount;

  @NotLogged private Debouncer m_jamDetect;

  public Shooter(ShooterIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_currentState = SHOOTER_STATE.STOP;

    this.m_velocity = RotationsPerSecond.of(0);
    this.m_hoodAngle = Degrees.of(0);

    this.m_isGoingTowardsAllianceZone = false;
    this.m_didIntake = false;

    this.m_fuelCount = 0;

    this.m_FPSLists = new ArrayList<Double>();
    this.m_FPSTimer = new Timer();
    this.m_FPSTimer.start();

    this.m_jamDetect = new Debouncer(ShooterConstants.JAM_DETECT_TIME);
  }

  public void periodic() {

    switch (this.m_currentState) {
      case SHOOT:
        this.m_velocity = ShooterMath.getCurrentSolution().flywheelSpeed;
        this.m_hoodAngle = ShooterMath.getCurrentSolution().hoodAngle;
        break;

      default:
        this.m_velocity = this.m_currentState.m_velocity;
        this.m_hoodAngle = this.m_currentState.m_hoodAngle;
        break;
    }

    // Fuel Tracking
    double totalTime = 0;
    for (int i = 0; i < this.m_FPSLists.size(); i++) {
      totalTime += this.m_FPSLists.get(i);
    }

    for (int i = 0; i < 100; i++) {
      if (totalTime <= 1) break;
      this.m_FPSLists.remove(this.m_FPSLists.size());
    }

    if (this.m_io.hasBreakerBroke() || this.m_io.hasBreakerFollowerBroke()) {
      if (this.m_FPSTimer.get() < 1) {
        this.m_FPSLists.add(this.m_FPSTimer.get());
      }
      this.m_FPSTimer.restart();

      this.m_fuelCount++;
    }

    this.m_io.setTargetVelocity(this.m_velocity);
    this.m_io.setHoodAngle(this.m_hoodAngle);

    this.m_io.update(this.m_currentState.m_subsystemPeriodicFrequency.in(Seconds));
  }

  @NotLogged
  public AngularVelocity getVelocity() {
    return this.m_io.getVelocity();
  }

  @NotLogged
  public AngularVelocity getTargetVelocity() {
    return this.m_velocity;
  }

  @NotLogged
  public boolean isAtTargetVelocity() {
    return this.getVelocity()
        .isNear(getTargetVelocity(), ShooterConstants.FLYWHEEL_TARGET_ERROR_RANGE);
  }

  @NotLogged
  public boolean isHoodAtTargetAngle() {
    return this.getHoodAngle()
        .isNear(getTargetHoodAngle(), ShooterConstants.HOOD_TARGET_ERROR_RANGE);
  }

  @NotLogged
  public Angle getTargetHoodAngle() {
    return this.m_hoodAngle;
  }

  @NotLogged
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
    IDLE(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(0)),
    SHOOT(Milliseconds.of(20), RotationsPerSecond.of(0), Degrees.of(0)),
    PRESET_SCORE(Milliseconds.of(60), RotationsPerSecond.of(65), Degrees.of(0));

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

  @NotLogged
  public SHOOTER_STATE getState() {
    return this.m_currentState;
  }

  @NotLogged
  public int getBallCount() {
    return this.m_fuelCount;
  }

  @NotLogged
  public void resetBallCount() {
    this.m_fuelCount = 0;
  }

  @NotLogged
  public double getFPS() {
    double totalTime = 0;
    for (int i = 0; i < this.m_FPSLists.size(); i++) {
      totalTime += this.m_FPSLists.get(i);
    }

    return totalTime / this.m_FPSLists.size();
  }

  @Logged(importance = Importance.INFO)
  public boolean isJammed() {
    return this.m_jamDetect.calculate(
        !this.m_io.hasBreakerBroke() && !this.m_io.hasBreakerFollowerBroke());
  }

  @Logged(importance = Importance.INFO)
  public boolean hasBreakerBroke() {
    return this.m_io.hasBreakerBroke();
  }

  @Logged(importance = Importance.INFO)
  public boolean hasBreakerFollowerBroke() {
    return this.m_io.hasBreakerFollowerBroke();
  }

  public void setFuelSim(FuelSim fuelSim) {
    this.m_io.setFuelSim(fuelSim);
  }
}
