package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import frc.robot.utils.FuelSim;
import frc.robot.utils.shooterMath.ShooterMath;
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
  private AngularVelocity m_velocity;

  @Logged(importance = Importance.CRITICAL)
  private Angle m_hoodAngle;

  @NotLogged private boolean m_isGoingTowardsAllianceZone;

  @Logged(importance = Importance.INFO)
  private boolean m_didIntake;

  @Logged(importance = Importance.INFO)
  private double m_FPS;

  @Logged(importance = Importance.CRITICAL)
  private final List<ArrayDeque<Double>> timestampQueues = new ArrayList<>(2);

  @NotLogged private Debouncer m_jamDetect;

  @Logged(importance = Importance.INFO)
  private Boolean[] fuelDetection = new Boolean[2];

  /*
   * This Matrix is derived from values of the beam breakers for rising and falling detection.
   *
   * [ currentLeft, previousLeft ]
   * [ currentRight, previousRight]
   */
  @NotLogged private Matrix<N2, N2> risingDetection;

  public Shooter(ShooterIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_currentState = SHOOTER_STATE.STOP;

    this.m_velocity = RotationsPerSecond.of(0);
    this.m_hoodAngle = Degrees.of(0);

    this.m_isGoingTowardsAllianceZone = false;
    this.m_didIntake = false;

    timestampQueues.add(new ArrayDeque<>()); // left
    timestampQueues.add(new ArrayDeque<>()); // right

    this.m_jamDetect = new Debouncer(ShooterConstants.JAM_DETECT_TIME);

    this.risingDetection = MatBuilder.fill(Nat.N2(), Nat.N2(), 0, 0, 0, 0);
  }

  public void periodic() {

    switch (this.m_currentState) {
      case SHOOT:
        this.m_velocity = RotationsPerSecond.of(ShooterMath.getInterpolatedRPS());
        this.m_hoodAngle = Degrees.of(ShooterMath.getInterpolatedAngle());
        calculateFPS();
        break;

      default:
        this.m_velocity = this.m_currentState.m_velocity;
        this.m_hoodAngle = this.m_currentState.m_hoodAngle;
        break;
    }
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

  public void calculateFPS() {
    /* Fuel per second Handling */

    // Pruning the list
    double currentTime = System.currentTimeMillis() / 1000.0;
    for (ArrayDeque<Double> queue : timestampQueues) {
      for (double fuel : queue) {
        if (fuel < currentTime - 1) {
          queue.remove(fuel);
        } else {
          break;
        }
      }
    }

    // Update rising detection matrix
    risingDetection.set(0, 0, m_io.hasBreakerBroke() ? 1 : 0);
    risingDetection.set(1, 0, m_io.hasBreakerFollowerBroke() ? 1 : 0);
    for (int i = 0; i < risingDetection.getNumCols(); i++) {
      fuelDetection[i] = (risingDetection.get(0, 0) > risingDetection.get(i, 1));
      if (fuelDetection[i]) {
        if ((currentTime - timestampQueues.get(i).getLast()) < 1 / 24) {
          continue;
        }
        timestampQueues.get(i).add(currentTime);
      }
    }

    // Calculate the FPS
    this.m_FPS =
        (timestampQueues.get(0).size() + timestampQueues.get(1).size())
            / (Math.max(timestampQueues.get(0).peekLast(), timestampQueues.get(1).peekLast())
                - Math.min(timestampQueues.get(0).peekFirst(), timestampQueues.get(1).peekFirst()));

    // Update matrix (Shift from left column to right column; reset first column)
    risingDetection.set(0, 1, risingDetection.get(0, 0));
    risingDetection.set(1, 1, risingDetection.get(1, 0));

    risingDetection.setColumn(0, MatBuilder.fill(Nat.N2(), Nat.N1(), 0.0));
  }

  @NotLogged
  public int getBallCount() {
    return this.timestampQueues.get(0).size() + this.timestampQueues.get(1).size();
  }

  @NotLogged
  public void resetBallCount() {
    this.timestampQueues.get(0).clear();
    this.timestampQueues.get(1).clear();
    this.m_FPS = 0.0;
  }

  @Logged(importance = Importance.INFO)
  public double getFPS() {
    return this.m_FPS;
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
