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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import frc.robot.utils.FuelSim;
import frc.robot.utils.shooterMath.ShooterMath2;
import java.util.ArrayDeque;

@Logged
public class Shooter {
  @Logged(importance = Importance.CRITICAL)
  private SHOOTER_STATE m_currentState;

  @Logged(importance = Importance.CRITICAL)
  private final ShooterIO m_io;

  @NotLogged private final TimesConsumer m_timesConsumer;

  @Logged(importance = Importance.CRITICAL)
  private AngularVelocity m_leftTargetVelocity;

  @Logged(importance = Importance.CRITICAL)
  private Angle m_leftHoodTarget;

  @Logged(importance = Importance.CRITICAL)
  private AngularVelocity m_rightTargetVelocity;

  @Logged(importance = Importance.CRITICAL)
  private Angle m_rightHoodTarget;

  @NotLogged private boolean m_isGoingTowardsAllianceZone;

  @Logged(importance = Importance.INFO)
  private boolean m_didIntake;

  @NotLogged private Timer m_FPSTimer;

  @Logged(importance = Importance.INFO)
  private double m_FPS;

  @NotLogged private ArrayDeque<Double> m_fpsDeque;

  @Logged(importance = Importance.DEBUG)
  private double m_fpsRunningSum = 0.0;

  @Logged(importance = Importance.CRITICAL)
  private int m_fuelCount;

  @NotLogged private Debouncer m_jamDetect;

  @NotLogged private boolean m_prevLeftBroken = false;

  @NotLogged private boolean m_prevRightBroken = false;

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

    this.m_leftTargetVelocity = RotationsPerSecond.of(0);
    this.m_leftHoodTarget = Degrees.of(0);
    this.m_rightTargetVelocity = RotationsPerSecond.of(0);
    this.m_rightHoodTarget = Degrees.of(0);

    this.m_isGoingTowardsAllianceZone = false;
    this.m_didIntake = false;

    this.m_fuelCount = 0;

    this.m_fpsDeque = new ArrayDeque<Double>();
    this.m_FPSTimer = new Timer();
    this.m_FPSTimer.start();

    this.m_jamDetect = new Debouncer(ShooterConstants.JAM_DETECT_TIME);

    SmartDashboard.putNumber("Flywheel Scalar", 2);
  }

  /** Runs periodic shooter logic including target tracking and fuel counting. */
  public void periodic() {

    switch (this.m_currentState) {
      case SHOOT:
        this.m_leftTargetVelocity =
            ShooterMath2.currentSolution
                .shooterLeft()
                .flywheelOmega()
                .times(SmartDashboard.getNumber("Flywheel Scalar", 1));
        this.m_leftHoodTarget = ShooterMath2.currentSolution.shooterLeft().hoodAngle();
        this.m_rightTargetVelocity =
            ShooterMath2.currentSolution
                .shooterRight()
                .flywheelOmega()
                .times(SmartDashboard.getNumber("Flywheel Scalar", 1));
        this.m_rightHoodTarget = ShooterMath2.currentSolution.shooterRight().hoodAngle();
        break;

      default:
        this.m_leftTargetVelocity = this.m_currentState.m_velocity;
        this.m_leftHoodTarget = this.m_currentState.m_hoodAngle;
        this.m_rightTargetVelocity = this.m_currentState.m_velocity;
        this.m_rightHoodTarget = this.m_currentState.m_hoodAngle;
        break;
    }

    // Fuel Counting
    boolean left = m_io.hasBreakerLeftBroke();
    boolean right = m_io.hasBreakerRightBroke();

    if ((left && !m_prevLeftBroken) || (right && !m_prevRightBroken)) {
      double elapsed = m_FPSTimer.get();
      if (elapsed < 1.0) addInterval(elapsed); // see deque suggestion below
      m_FPSTimer.restart();
      m_fuelCount++;
    }
    m_prevLeftBroken = left;
    m_prevRightBroken = right;

    this.m_io.setLeftTargetVelocity(this.m_leftTargetVelocity);
    this.m_io.setLeftHoodTarget(this.m_leftHoodTarget);
    this.m_io.setRightTargetVelocity(this.m_rightTargetVelocity);
    this.m_io.setRightHoodTarget(this.m_rightHoodTarget);

    this.m_io.update(this.m_currentState.m_subsystemPeriodicFrequency.in(Seconds));
  }

  /** Returns whether the left flywheel is at its target velocity. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isAtLeftTargetVelocity() {
    return Mode.currentMode == CurrentMode.REAL
        ? (ShooterMath2.currentSolution.shooterLeft().inTolerance(this.getLeftVelocity()))
        : true;
  }

  /** Returns whether the left hood is at its target angle. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isHoodAtLeftTargetAngle() {
    return Mode.currentMode == CurrentMode.REAL
        ? (ShooterMath2.currentSolution.shooterLeft().inTolerance(this.getLeftHoodPosition()))
        : true;
  }

  /** Returns whether the right flywheel is at its target velocity. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isAtRightTargetVelocity() {
    return Mode.currentMode == CurrentMode.REAL
        ? (ShooterMath2.currentSolution.shooterRight().inTolerance(this.getRightVelocity()))
        : true;
  }

  /** Returns whether the right hood is at its target angle. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isHoodAtRightTargetAngle() {
    return Mode.currentMode == CurrentMode.REAL
        ? (ShooterMath2.currentSolution.shooterRight().inTolerance(this.getRightHoodPosition()))
        : true;
  }

  /** Returns the left target hood angle. */
  @NotLogged
  public Angle getLeftTargetHood() {
    return this.m_leftHoodTarget;
  }

  /** Returns the right target hood angle. */
  @NotLogged
  public Angle getRightTargetHood() {
    return this.m_rightHoodTarget;
  }

  /** Returns the left hood position. */
  @NotLogged
  public Angle getLeftHoodPosition() {
    return this.m_io.getLeftHoodPosition();
  }

  /** Returns the right hood position. */
  @NotLogged
  public Angle getRightHoodPosition() {
    return this.m_io.getRightHoodPosition();
  }

  /** Returns the left target velocity. */
  @NotLogged
  public AngularVelocity getLeftTargetVelocity() {
    return this.m_leftTargetVelocity;
  }

  /** Returns the right target velocity. */
  @NotLogged
  public AngularVelocity getRightTargetVelocity() {
    return this.m_rightTargetVelocity;
  }

  /** Returns the left flywheel velocity. */
  @NotLogged
  public AngularVelocity getLeftVelocity() {
    return this.m_io.getLeftVelocity();
  }

  /** Returns the right flywheel velocity. */
  @NotLogged
  public AngularVelocity getRightVelocity() {
    return this.m_io.getRightVelocity();
  }

  /**
   * Sets whether the robot is going towards the alliance zone.
   *
   * @param isGoingTowardsAllianceZone true if heading towards alliance zone
   */
  public void setGoingTowardsAllianceZone(boolean isGoingTowardsAllianceZone) {
    this.m_isGoingTowardsAllianceZone = isGoingTowardsAllianceZone;
  }

  /**
   * Sets whether the robot has intaked.
   *
   * @param didIntake true if intake has occurred
   */
  public void setDidIntake(boolean didIntake) {
    this.m_didIntake = didIntake;
  }

  public enum SHOOTER_STATE {
    STOP(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(0)),
    IDLE(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(0)),
    SHOOT(Milliseconds.of(20), RotationsPerSecond.of(0), Degrees.of(0)),
    PRESET_SCORE(Milliseconds.of(20), RotationsPerSecond.of(65), Degrees.of(0));

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

  /** Returns the current shooter state. */
  @NotLogged
  public SHOOTER_STATE getState() {
    return this.m_currentState;
  }

  /** Returns the fuel count. */
  @NotLogged
  public int getBallCount() {
    return this.m_fuelCount;
  }

  /** Resets the fuel count to zero. */
  @NotLogged
  public void resetBallCount() {
    this.m_fuelCount = 0;
  }

  void addInterval(double interval) {
    m_fpsDeque.addLast(interval);
    m_fpsRunningSum += interval;
    while (m_fpsRunningSum > 1.0 && !m_fpsDeque.isEmpty()) {
      m_fpsRunningSum -= m_fpsDeque.removeFirst();
    }
  }

  /** Returns the fuel per second rate. */
  @NotLogged
  public double getFPS() {
    if (Mode.currentMode == CurrentMode.SIMULATION) {
      return 0.3;
    }

    if (this.m_fpsDeque.isEmpty() || this.m_fpsRunningSum <= 0.0) {
      return 0.0;
    }

    // FPS = number of events / total time window (events per second)
    double fps = this.m_fpsDeque.size() / this.m_fpsRunningSum;
    this.m_FPS = fps;
    return fps;
  }

  /** Returns whether the shooter is jammed. */
  @Logged(importance = Importance.INFO)
  public boolean isJammed() {
    return this.m_jamDetect.calculate(
        !this.m_io.hasBreakerLeftBroke() && !this.m_io.hasBreakerRightBroke());
  }

  /** Returns whether the left beam breaker has been broken. */
  @Logged(importance = Importance.INFO)
  public boolean hasBreakerLeftBroke() {
    return this.m_io.hasBreakerLeftBroke();
  }

  /** Returns whether the right beam breaker has been broken. */
  @Logged(importance = Importance.INFO)
  public boolean hasBreakerRightBroke() {
    return this.m_io.hasBreakerRightBroke();
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
