package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import frc.robot.utils.FuelSim;
import frc.robot.utils.shooterMath.ShooterMath3;

/** Shooter subsystem state machine, control targets, and beam-break based fuel accounting. */
@Logged
public class Shooter {
  @Logged(importance = Importance.CRITICAL)
  private SHOOTER_STATE m_currentState;

  @Logged(importance = Importance.CRITICAL)
  private final ShooterIO m_io;

  @NotLogged private final TimesConsumer m_timesConsumer;

  @Logged(importance = Importance.CRITICAL)
  private AngularVelocity m_targetVelocity;

  @Logged(importance = Importance.CRITICAL)
  private Angle m_hoodTarget;

  @NotLogged private boolean m_testing = false;

  @Logged(importance = Importance.INFO)
  private boolean m_shouldOverride;

  /**
   * Constructs a new Shooter.
   *
   * @param io the shooter IO implementation
   * @param consumer the times consumer for dynamic scheduling
   */
  public Shooter(ShooterIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_currentState = SHOOTER_STATE.STOP;

    this.m_targetVelocity = RotationsPerSecond.of(0);
    this.m_hoodTarget = Degrees.of(0);

    this.m_shouldOverride = false;

    SmartDashboard.putNumber("tuning/preferredMinArrivalAngleDeg", 0.0);
    SmartDashboard.putNumber("tuning/speedTransferEfficiency", 0.0);
  }

  /** Runs periodic shooter logic including target tracking and fuel counting. */
  public void periodic() {
    switch (this.m_currentState) {
      case SHOOT:
        this.m_targetVelocity = ShooterMath3.currentSolution.flywheelOmega();
        this.m_hoodTarget = ShooterMath3.currentSolution.hoodAngle();
        break;
      default:
        this.m_targetVelocity = this.m_currentState.m_velocity;
        this.m_hoodTarget = this.m_currentState.m_hoodAngle;
        break;
    }

    this.m_io.setTargetVelocity(this.m_targetVelocity);
    this.m_io.setHoodTarget(this.m_hoodTarget);

    this.m_io.update(this.m_currentState.m_subsystemPeriodicFrequency.in(Seconds));
  }

  /** Returns whether the flywheel is at its target velocity. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isAtTargetVelocity() {
    return Mode.currentMode == CurrentMode.REAL
        ? (this.m_targetVelocity.isNear(
            this.getVelocity(), ShooterConstants.FLYWHEEL_TARGET_ERROR_RANGE))
        : true;
  }

  /** Returns whether the hood is at its target angle. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isHoodAtTargetAngle() {
    return Mode.currentMode == CurrentMode.REAL
        ? (this.m_hoodTarget.isNear(
            this.getHoodPosition(), ShooterConstants.HOOD_TARGET_ERROR_RANGE))
        : true;
  }

  /** Returns the target hood angle. */
  @NotLogged
  public Angle getHoodTarget() {
    return this.m_hoodTarget;
  }

  /** Returns the hood position. */
  @NotLogged
  public Angle getHoodPosition() {
    return this.m_io.getHoodPosition();
  }

  /** Returns the target velocity. */
  @NotLogged
  public AngularVelocity getTargetVelocity() {
    return this.m_targetVelocity;
  }

  /** Returns the flywheel velocity. */
  @NotLogged
  public AngularVelocity getVelocity() {
    return this.m_io.getVelocity();
  }

  public enum SHOOTER_STATE {
    STOP(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(ShooterConstants.HOOD_MIN)),
    IDLE(
        Milliseconds.of(60),
        RotationsPerSecond.of(0), // TODO: tune for fastest without drawing too much current
        Degrees.of(ShooterConstants.HOOD_MIN)),
    SHOOT(Milliseconds.of(20), RotationsPerSecond.of(0), Degrees.of(ShooterConstants.HOOD_MIN)),
    TESTING(
        Milliseconds.of(20),
        RotationsPerSecond.of(40),
        Degrees.of((ShooterConstants.HOOD_MAX + ShooterConstants.HOOD_MIN) / 2)),
    TRENCH(
        Milliseconds.of(20),
        RotationsPerSecond.of(0),
        Degrees.of(ShooterConstants.HOOD_MIN)), // TODO: tune presets
    CORNER(
        Milliseconds.of(20),
        RotationsPerSecond.of(0),
        Degrees.of(ShooterConstants.HOOD_MIN)), // TODO: tune presets
    TOWER(
        Milliseconds.of(20),
        RotationsPerSecond.of(0),
        Degrees.of(ShooterConstants.HOOD_MIN)); // TODO: tune presets

    private final Time m_subsystemPeriodicFrequency;
    private final AngularVelocity m_velocity;
    private final Angle m_hoodAngle;

    SHOOTER_STATE(Time subsystemPeriodicFrequency, AngularVelocity velocity, Angle hoodAngle) {
      this.m_subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.m_velocity = velocity;
      this.m_hoodAngle = hoodAngle;
    }
  }

  /**
   * Sets the current shooter state.
   *
   * @param pState the shooter state to set
   */
  public void setState(SHOOTER_STATE pState) {
    if (this.m_testing) return;
    if (this.m_shouldOverride) return;
    if (!this.m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        pState.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Shooter, pState.m_subsystemPeriodicFrequency);
    }
    this.m_currentState = pState;
  }

  /**
   * Sets the current shooter state for testing mode only.
   *
   * @param pState the shooter state to set
   */
  public void setStateTesting(SHOOTER_STATE pState) {
    if (!this.m_testing) return;
    if (!this.m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        pState.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Shooter, pState.m_subsystemPeriodicFrequency);
    }
    this.m_currentState = pState;
  }

  /**
   * Overrides the shooter state.
   *
   * @param pShouldOverride true to enable override
   * @param pState the shooter state to override with
   */
  public void override(boolean pShouldOverride, SHOOTER_STATE pState) {
    if (this.m_testing) return;
    this.m_shouldOverride = pShouldOverride;
    if (!this.m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        pState.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Shooter, pState.m_subsystemPeriodicFrequency);
    }
    this.m_currentState = pState;
  }

  /**
   * Enables or disables testing mode.
   *
   * @param testing true to enable testing mode
   */
  public void setTesting(boolean testing) {
    this.m_testing = testing;
  }

  /** Returns the current shooter state. */
  @NotLogged
  public SHOOTER_STATE getState() {
    return this.m_currentState;
  }

  /**
   * Sets the fuel simulation reference.
   *
   * @param fuelSim the fuel simulation instance
   */
  public void setFuelSim(FuelSim fuelSim) {
    this.m_io.setFuelSim(fuelSim);
  }
}
