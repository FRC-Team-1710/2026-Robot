package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Robot;
import frc.robot.constants.SubsystemConstants.IndexerConstants;

public class Indexer {

  private final IndexerIO m_io;

  private IndexerStates m_currentState = IndexerStates.Idle;

  private boolean m_testing = false;

  private boolean m_brownoutMode = false;

  /**
   * Creates a new Indexer.
   *
   * @param io the indexer IO implementation
   */
  public Indexer(IndexerIO io) {
    m_io = io;
  }

  public void toggleBrownout() {
    m_brownoutMode = !m_brownoutMode;
    Robot.telemetry().log("Brownout/Indexer", m_brownoutMode);
  }

  /**
   * Sets the current indexer state.
   *
   * @param state the indexer state to set
   */
  public void setState(IndexerStates state) {
    if (m_testing) return;
    m_currentState = state;
    m_io.setVoltage(m_brownoutMode ? m_currentState.brownoutVoltage : m_currentState.voltage);
  }

  /**
   * Sets the current indexer state for testing mode only.
   *
   * @param state the indexer state to set
   */
  public void setStateTesting(IndexerStates state) {
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

  public enum IndexerStates {
    Idle(),
    Run(IndexerConstants.Software.kRunVoltage, IndexerConstants.Software.kBrownoutRunVoltage);

    public final Voltage voltage;
    public final Voltage brownoutVoltage;

    IndexerStates() {
      this(Volts.of(0), Volts.of(0));
    }

    IndexerStates(Voltage voltage, Voltage brownoutVoltage) {
      this.voltage = voltage;
      this.brownoutVoltage = brownoutVoltage;
    }
  }
}
