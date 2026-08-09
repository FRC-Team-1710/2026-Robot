package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.SubsystemConstants.FeederConstants;

public class Feeder {

  private FeederStates m_currentState = FeederStates.Idle;

  private final FeederIO m_io;

  private boolean m_testing = false;

  private boolean m_brownoutMode = false;

  /**
   * Constructs a new Feeder.
   *
   * @param io the feeder IO implementation
   */
  public Feeder(FeederIO io) {
    m_io = io;
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
    m_io.setVoltage(m_brownoutMode ? m_currentState.brownoutVoltage : m_currentState.voltage);
  }

  /**
   * Sets the current feeder state for testing mode only.
   *
   * @param state the feeder state to set
   */
  public void setStateTesting(FeederStates state) {
    if (!m_testing) return;
    m_currentState = state;
    m_io.setVoltage(m_currentState.voltage);
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
