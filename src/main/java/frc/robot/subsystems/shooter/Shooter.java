package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.numbers.N2;
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
import frc.robot.utils.shooterMath.ShooterMath2;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

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

  @Logged(importance = Importance.INFO)
  private double m_fps;

  @NotLogged private boolean m_testing = false;

  @NotLogged private final List<ArrayDeque<Long>> m_timestampQueues = new ArrayList<>(2);

  @NotLogged private Debouncer m_jamDetect;

  @NotLogged private Boolean[] m_fuelDetection = new Boolean[] {false, false};

  @Logged(importance = Importance.INFO)
  private int m_ballCount;

  /**
   * This Matrix is derived from values of the beam breakers for rising and falling detection.
   *
   * <p>Structure: <code>[currentLeft , previousLeft] [currentRight, previousRight]</code>
   *
   * <p>Stored as ints: 1 = beam broken, 0 = beam clear.
   */
  @NotLogged private Matrix<N2, N2> m_risingDetection;

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

    this.m_leftTargetVelocity = RotationsPerSecond.of(0);
    this.m_leftHoodTarget = Degrees.of(0);
    this.m_rightTargetVelocity = RotationsPerSecond.of(0);
    this.m_rightHoodTarget = Degrees.of(0);

    this.m_isGoingTowardsAllianceZone = false;
    this.m_didIntake = false;

    this.m_ballCount = 0;

    this.m_risingDetection = MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 0, 0, 0);

    m_timestampQueues.add(new ArrayDeque<>()); // left
    m_timestampQueues.add(new ArrayDeque<>()); // right

    this.m_jamDetect = new Debouncer(ShooterConstants.JAM_DETECT_TIME);

    this.m_shouldOverride = false;

    SmartDashboard.putNumber("preferredMinArrivalAngleDeg", 35);
    SmartDashboard.putNumber("speedTransferEfficiency", 0.54375);
  }

  /** Runs periodic shooter logic including target tracking and fuel counting. */
  public void periodic() {

    switch (this.m_currentState) {
      case SHOOT:
        this.m_leftTargetVelocity = ShooterMath2.currentSolution.shooterLeft().flywheelOmega();
        this.m_leftHoodTarget = ShooterMath2.currentSolution.shooterLeft().hoodAngle();
        this.m_rightTargetVelocity = ShooterMath2.currentSolution.shooterRight().flywheelOmega();
        this.m_rightHoodTarget = ShooterMath2.currentSolution.shooterRight().hoodAngle();
        break;

      default:
        this.m_leftTargetVelocity = this.m_currentState.m_velocity;
        this.m_leftHoodTarget = this.m_currentState.m_hoodAngle;
        this.m_rightTargetVelocity = this.m_currentState.m_velocity;
        this.m_rightHoodTarget = this.m_currentState.m_hoodAngle;
        break;
    }

    this.m_io.setLeftTargetVelocity(this.m_leftTargetVelocity);
    this.m_io.setLeftHoodTarget(this.m_leftHoodTarget);
    this.m_io.setRightTargetVelocity(this.m_rightTargetVelocity);
    this.m_io.setRightHoodTarget(this.m_rightHoodTarget);

    this.m_io.update(this.m_currentState.m_subsystemPeriodicFrequency.in(Seconds));

    // Always run FPS tracking regardless of state so pruning stays current
    calculateFPS();
  }

  /** Returns whether the left flywheel is at its target velocity. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isAtLeftTargetVelocity() {
    return Mode.currentMode == CurrentMode.REAL
        ? (ShooterMath2.currentSolution
            .shooterLeft()
            .flywheelOmega()
            .isNear(this.getLeftVelocity(), RotationsPerSecond.of(20)))
        : true;
  }

  /** Returns whether the left hood is at its target angle. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isHoodAtLeftTargetAngle() {
    return Mode.currentMode == CurrentMode.REAL
        ? (ShooterMath2.currentSolution
            .shooterLeft()
            .hoodAngle()
            .isNear(this.getLeftHoodPosition(), Degrees.of(3)))
        : true;
  }

  /** Returns whether the right flywheel is at its target velocity. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isAtRightTargetVelocity() {
    return Mode.currentMode == CurrentMode.REAL
        ? (ShooterMath2.currentSolution
            .shooterRight()
            .flywheelOmega()
            .isNear(this.getRightVelocity(), RotationsPerSecond.of(20)))
        : true;
  }

  /** Returns whether the right hood is at its target angle. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isHoodAtRightTargetAngle() {
    return Mode.currentMode == CurrentMode.REAL
        ? (ShooterMath2.currentSolution
            .shooterRight()
            .hoodAngle()
            .isNear(this.getRightHoodPosition(), Degrees.of(3)))
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
    STOP(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(ShooterConstants.HOOD_MIN)),
    IDLE(Milliseconds.of(60), RotationsPerSecond.of(0), Degrees.of(ShooterConstants.HOOD_MIN)),
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

  /** Calculates the fuel per second (FPS) based on the timestamps of fuel detection events. */
  @NotLogged
  public void calculateFPS() {
    /* Fuel per second Handling */
    long currentTime = System.currentTimeMillis();

    // Prune timestamps older than 1 second using an iterator (safe removal while iterating)
    for (ArrayDeque<Long> queue : m_timestampQueues) {
      while (!queue.isEmpty() && queue.peekFirst() < currentTime - 1000) {
        queue.removeFirst();
      }
    }

    // Shift current column → previous column in rising detection matrix
    m_risingDetection.set(0, 1, m_risingDetection.get(0, 0));
    m_risingDetection.set(1, 1, m_risingDetection.get(1, 0));

    // Update current column with fresh sensor readings
    m_risingDetection.set(0, 0, m_io.hasBreakerLeftBroke() ? 1 : 0);
    m_risingDetection.set(1, 0, m_io.hasBreakerRightBroke() ? 1 : 0);
    // Rising edge: current=1, previous=0 → use per-row index for each side
    for (int i = 0; i < 2; i++) {
      m_fuelDetection[i] = (m_risingDetection.get(i, 0) > m_risingDetection.get(i, 1));
      if (m_fuelDetection[i]) {
        ArrayDeque<Long> queue = m_timestampQueues.get(i);
        // Debounce: ignore duplicate triggers within a physically impossible interval (< 1/24 s)
        if (!queue.isEmpty()
            && (currentTime - queue.peekLast()) < (ShooterConstants.UNREALISTIC_FPS_INTERVAL_MS)) {
          continue;
        }
        queue.addLast(currentTime);
        this.m_ballCount++;
      }
    }

    // Calculate combined FPS over the 1-second window
    int totalEvents = m_timestampQueues.get(0).size() + m_timestampQueues.get(1).size();
    if (totalEvents < 2) {
      // Not enough data points — keep last valid FPS or return 0
      if (totalEvents == 0) this.m_fps = 0.0;
      return;
    }

    // Span = from earliest event across both queues to latest
    double span =
        (Math.max(
                m_timestampQueues.get(0).isEmpty()
                    ? Double.MIN_VALUE
                    : m_timestampQueues.get(0).peekLast(),
                m_timestampQueues.get(1).isEmpty()
                    ? Double.MIN_VALUE
                    : m_timestampQueues.get(1).peekLast()))
            - 1000;
    this.m_FPS = (span > 0) ? (totalEvents * 1000.0 / span) : 0.0;
  }

  /** Returns the current ball count based on the timestamps of fuel detection events. */
  @NotLogged
  public int getBallCount() {
    return this.m_ballCount;
  }

  /** Resets the ball count and FPS to zero. */
  @NotLogged
  public void resetBallCount() {
    this.m_timestampQueues.get(0).clear();
    this.m_timestampQueues.get(1).clear();
    this.m_ballCount = 0;
    this.m_fps = 0.0;
  }

  /** Returns the current fuel per second (FPS) based on the timestamps of fuel detection events. */
  @Logged(importance = Importance.INFO)
  public double getFPS() {
    return this.m_fps;
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
