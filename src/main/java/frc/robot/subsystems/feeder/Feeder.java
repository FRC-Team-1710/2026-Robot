package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.SubsystemConstants.FeederConstants;

@Logged
public class Feeder {

  @Logged(importance = Importance.CRITICAL)
  private FeederStates m_currentState = FeederStates.Idle;

  @NotLogged private final BooleanSupplier m_bumpSupplier;

  @Logged(importance = Importance.CRITICAL)
  private final FeederIO m_io;

  @NotLogged private boolean m_testing = false;

  @NotLogged private boolean m_brownoutMode = false;

  /**
   * Constructs a new Feeder.
   *
   * @param io the feeder IO implementation
   * @param bumpSupplier the bump supplier
   */
  public Feeder(FeederIO io, BooleanSupplier bumpSupplier) {
    m_io = io;
    m_bumpSupplier = bumpSupplier;
  }

  public void toggleBrownout() {
    m_brownoutMode = !m_brownoutMode;
    Robot.telemetry().log("Brownout/Feeder", m_brownoutMode);
  }

  /**
   * Sets the current feeder state.
   *
   * @param state the feeder state to set
   */
  public void setState(FeederStates state) {
    if (m_testing) return;
    m_currentState = state;
    m_io.setVoltage(m_bumpSupplier.getAsBoolean() ? FeederConstants.Software.kReverseVoltage : (m_brownoutMode ? m_currentState.brownoutVoltage : m_currentState.voltage));
  }

  /**
   * Sets the current feeder state for testing mode only.
   *
   * @param state the feeder state to set
   */
  public void setStateTesting(FeederStates state) {
    if (!m_testing) return;
    m_currentState = state;
    m_io.setVoltage(m_bumpSupplier.getAsBoolean() ? FeederConstants.Software.kReverseVoltage : (m_currentState.voltage));
  }

  /**
   * Enables or disables testing mode.
   *
   * @param testing true to enable testing mode
   */
  public void setTesting(boolean testing) {
    m_testing = testing;
  }

  /** Returns the current feeder state. */
  @NotLogged
  public FeederStates getState() {
    return m_currentState;
  }

  public enum FeederStates {
    Idle(),
    Run(FeederConstants.Software.kRunVoltage, FeederConstants.Software.kBrownoutRunVoltage);

    public final Voltage voltage;
    public final Voltage brownoutVoltage;

    FeederStates() {
      this(Volts.of(0), Volts.of(0));
    }

    FeederStates(Voltage voltage, Voltage brownoutVoltage) {
      this.voltage = voltage;
      this.brownoutVoltage = brownoutVoltage;
    }
  }
}
