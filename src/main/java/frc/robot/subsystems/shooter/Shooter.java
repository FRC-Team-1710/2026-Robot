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

  @NotLogged private ArrayDeque<Double> fpsDeque;

  @NotLogged private double fpsRunningSum = 0.0;

  @Logged(importance = Importance.CRITICAL)
  private int m_fuelCount;

  @NotLogged private Debouncer m_jamDetect;

  @NotLogged private boolean prevLeftBroken = false;

  @NotLogged private boolean prevRightBroken = false;

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

    this.fpsDeque = new ArrayDeque<Double>();
    this.m_FPSTimer = new Timer();
    this.m_FPSTimer.start();

    this.m_jamDetect = new Debouncer(ShooterConstants.JAM_DETECT_TIME);

    SmartDashboard.putNumber("Flywheel Scalar", 2);
  }

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

    if ((left && !prevLeftBroken) || (right && !prevRightBroken)) {
      double elapsed = m_FPSTimer.get();
      if (elapsed < 1.0) fpsDeque.addLast(elapsed); // see deque suggestion below
      m_FPSTimer.restart();
      m_fuelCount++;
    }
    prevLeftBroken = left;
    prevRightBroken = right;

    this.m_io.setLeftTargetVelocity(this.m_leftTargetVelocity);
    this.m_io.setLeftHoodTarget(this.m_leftHoodTarget);
    this.m_io.setRightTargetVelocity(this.m_rightTargetVelocity);
    this.m_io.setRightHoodTarget(this.m_rightHoodTarget);

    this.m_io.update(this.m_currentState.m_subsystemPeriodicFrequency.in(Seconds));
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean isAtTargetVelocity() {
    return Mode.currentMode == CurrentMode.REAL
        ? (this.getLeftVelocity()
                .isNear(getLeftTargetVelocity(), ShooterConstants.FLYWHEEL_TARGET_ERROR_RANGE)
            && this.getRightVelocity()
                .isNear(getRightTargetVelocity(), ShooterConstants.FLYWHEEL_TARGET_ERROR_RANGE))
        : true;
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean isHoodAtTargetAngle() {
    return Mode.currentMode == CurrentMode.REAL
        ? (this.getLeftHoodPosition()
                .isNear(getLeftTargetHood(), ShooterConstants.HOOD_TARGET_ERROR_RANGE)
            && this.getRightHoodPosition()
                .isNear(getRightTargetHood(), ShooterConstants.HOOD_TARGET_ERROR_RANGE))
        : true;
  }

  @NotLogged
  public Angle getLeftTargetHood() {
    return this.m_leftHoodTarget;
  }

  @NotLogged
  public Angle getRightTargetHood() {
    return this.m_rightHoodTarget;
  }

  @NotLogged
  public Angle getLeftHoodPosition() {
    return this.m_io.getLeftHoodPosition();
  }

  @NotLogged
  public Angle getRightHoodPosition() {
    return this.m_io.getRightHoodPosition();
  }

  @NotLogged
  public AngularVelocity getLeftTargetVelocity() {
    return this.m_leftTargetVelocity;
  }

  @NotLogged
  public AngularVelocity getRightTargetVelocity() {
    return this.m_rightTargetVelocity;
  }

  @NotLogged
  public AngularVelocity getLeftVelocity() {
    return this.m_io.getLeftVelocity();
  }

  @NotLogged
  public AngularVelocity getRightVelocity() {
    return this.m_io.getRightVelocity();
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

  void addInterval(double interval) {
    fpsDeque.addLast(interval);
    fpsRunningSum += interval;
    while (fpsRunningSum > 1.0 && !fpsDeque.isEmpty()) {
      fpsRunningSum -= fpsDeque.removeFirst();
    }
  }

  @NotLogged
  public double getFPS() {
    double totalTime = 0;
    for (double item : this.fpsDeque) {
      totalTime += item;
    }

    return Mode.currentMode == CurrentMode.SIMULATION
        ? 0.3
        : totalTime / (this.fpsDeque.getLast() - this.fpsDeque.getFirst());
  }

  @Logged(importance = Importance.INFO)
  public boolean isJammed() {
    return this.m_jamDetect.calculate(
        !this.m_io.hasBreakerLeftBroke() && !this.m_io.hasBreakerRightBroke());
  }

  @Logged(importance = Importance.INFO)
  public boolean hasBreakerLeftBroke() {
    return this.m_io.hasBreakerLeftBroke();
  }

  @Logged(importance = Importance.INFO)
  public boolean hasBreakerRightBroke() {
    return this.m_io.hasBreakerRightBroke();
  }
}
