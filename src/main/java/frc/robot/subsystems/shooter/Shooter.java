package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.constants.SubsystemConstants.ShooterConstants;
import frc.robot.utils.FuelSim;
import frc.robot.utils.shooterMath.ShooterMath;

@Logged
public class Shooter {

  @Logged(importance = Importance.CRITICAL)
  private ShooterStates m_currentState = ShooterStates.IdleScore;

  @NotLogged private ShooterStates m_prevState = null;

  @NotLogged private final ShooterIO m_io;

  @NotLogged private AngularVelocity m_targetVelocity = RotationsPerSecond.of(0);

  @NotLogged private Angle m_hoodTarget = Degrees.of(0);

  @NotLogged private boolean m_testing = false;

  @NotLogged private boolean m_shouldOverride = false;

  /**
   * Constructs a new Shooter.
   *
   * @param io the shooter IO implementation
   */
  public Shooter(ShooterIO io) {
    m_io = io;
  }

  /** Runs periodic shooter logic including target tracking and fuel counting. */
  private void runMotors() {
    if (m_currentState != m_prevState) {
      if (m_prevState == null || m_prevState.highCurrent != m_currentState.highCurrent) {
        m_io.dynamicCurrentLimit(
            m_currentState.highCurrent
                ? ShooterConstants.Flywheel.Software.CurrentLimits.kHighSupply
                : ShooterConstants.Flywheel.Software.CurrentLimits.kLowSupply,
            m_currentState.highCurrent
                ? ShooterConstants.Flywheel.Software.CurrentLimits.kHighStator
                : ShooterConstants.Flywheel.Software.CurrentLimits.kLowStator);
      }
      m_prevState = m_currentState;
    }

    switch (m_currentState) {
      case Score:
        m_hoodTarget = ShooterMath.currentSolution.hoodAngle();
        break;
      case Pass:
        m_hoodTarget = ShooterMath.currentPassingSolution.hoodAngle();
        break;
      default:
        m_hoodTarget = m_currentState.hoodAngle;
        break;
    }

    switch (m_currentState) {
      case Score, IdleScore:
        m_targetVelocity = ShooterMath.currentSolution.flywheelOmega();
        break;
      case Pass, IdlePass:
        m_targetVelocity = ShooterMath.currentPassingSolution.flywheelOmega();
        break;
      default:
        m_targetVelocity = m_currentState.velocity;
        break;
    }

    m_io.setHoodTarget(m_hoodTarget);
    m_io.setTargetVelocity(m_targetVelocity);
  }

  /** Returns whether the hood is at its target angle. */
  @Logged(importance = Importance.CRITICAL)
  public boolean isHoodAtTargetAngle() {
    return Mode.currentMode == CurrentMode.REAL
        ? m_hoodTarget.isNear(
            m_io.getHoodPosition(), ShooterConstants.Hood.Software.kMaxHoodTargetError)
        : true;
  }

  /**
   * Sets the current shooter state.
   *
   * @param state the shooter state to set
   */
  public void setState(ShooterStates state) {
    if (m_testing) return;
    if (m_shouldOverride) return; // known shot locations
    m_currentState = state;
    runMotors();
  }

  /**
   * Sets the current shooter state for testing mode only.
   *
   * @param state the shooter state to set
   */
  public void setStateTesting(ShooterStates state) {
    if (!m_testing) return;
    m_currentState = state;
    runMotors();
  }

  /**
   * Overrides the shooter state.
   *
   * @param shouldOverride true to enable override
   * @param state the shooter state to override with
   */
  public void override(boolean shouldOverride, ShooterStates state) {
    if (m_testing) return;
    m_shouldOverride = shouldOverride;
    m_currentState = state;
    runMotors();
  }

  /**
   * Enables or disables testing mode.
   *
   * @param testing true to enable testing mode
   */
  public void setTesting(boolean testing) {
    m_testing = testing;
  }

  /** Returns the current shooter state. */
  @NotLogged
  public ShooterStates getState() {
    return m_currentState;
  }

  /**
   * Sets the fuel simulation reference.
   *
   * @param fuelSim the fuel simulation instance
   */
  public void setFuelSim(FuelSim fuelSim) {
    m_io.setFuelSim(fuelSim);
  }

  public enum ShooterStates {
    Stop(), // Used for testing only, not a real state
    AutoPreset(ShooterConstants.kAutoPresetDistance), // Preset for auto. Know distance from hub
    IdleScore(ShooterConstants.Hood.Hardware.kHoodMin),
    IdlePass(ShooterConstants.Hood.Hardware.kHoodMin),
    Score(true),
    Pass(true),
    Test(
        true,
        ShooterConstants.Hood.Hardware.kHoodMin
            .plus(ShooterConstants.Hood.Hardware.kHoodMax)
            .div(2.0),
        ShooterConstants.Flywheel.Software.kTestSlowVelocity),
    TestFast(
        true,
        ShooterConstants.Hood.Hardware.kHoodMin
            .plus(ShooterConstants.Hood.Hardware.kHoodMax)
            .div(2.0),
        ShooterConstants.Flywheel.Software.kTestFastVelocity),
    Trench(true, ShooterConstants.kTrenchPresetDistance),
    Tower(true, ShooterConstants.kTowerPresetDistance),
    TowerLeft(true, ShooterConstants.kTowerLeftPresetDistance),
    TowerRight(true, ShooterConstants.kTowerRightPresetDistance);

    public final boolean highCurrent;
    public final Angle hoodAngle;
    public final AngularVelocity velocity;

    ShooterStates() {
      this(false, ShooterConstants.Hood.Hardware.kHoodMin, RotationsPerSecond.of(0));
    }

    ShooterStates(boolean highCurrent) {
      this(highCurrent, ShooterConstants.Hood.Hardware.kHoodMin, RotationsPerSecond.of(0));
    }

    ShooterStates(Distance distance) {
      this(false, distance);
    }

    ShooterStates(boolean highCurrent, Distance distance) {
      this( // yes, we calculate it twice. deal with it
          highCurrent,
          ShooterMath.getScoreSolutionForDistance(distance).hoodAngle(),
          ShooterMath.getScoreSolutionForDistance(distance).flywheelOmega());
    }

    ShooterStates(Angle hoodAngle) {
      this(false, hoodAngle, RotationsPerSecond.of(0));
    }

    ShooterStates(Angle hoodAngle, AngularVelocity velocity) {
      this(false, hoodAngle, velocity);
    }

    ShooterStates(boolean highCurrent, Angle hoodAngle, AngularVelocity velocity) {
      this.highCurrent = highCurrent;
      this.hoodAngle = hoodAngle;
      this.velocity = velocity;
    }
  }
}
