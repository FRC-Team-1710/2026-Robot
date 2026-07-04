package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import org.littletonrobotics.junction.Logger;

// This class is made for the feeder subsystem which is a motor that moves fuel from the hopper into the shooter

public class Feeder {
  // Epilogue annotations removed; using Logger.recordOutput explicitly
  private FEEDER_STATE m_currentState;

  private final FeederIO m_io; //this is the IO implementation for the feeder which is used to set the motor output and update the subsystem

  private boolean m_testing = false; //changes wether the subsystem is in testing mode or not, which alloes for different states to be used for testing

  private final TimesConsumer m_timesConsumer; 

  /**
   * Constructs a new Feeder.
   *
   * @param io the feeder IO implementation
   * @param consumer the times consumer for dynamic scheduling
   */
  public Feeder(FeederIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_currentState = FEEDER_STATE.STOP; // Default state is to be stopped.
    Logger.recordOutput("Feeder/State", m_currentState);
  }

  /** Runs periodic feeder logic based on the current state.
   * Updates the feeder's behavior according to its current state.
   */
  public void periodic() {
    double output = this.m_currentState.m_velocity;
  
    //This is currently not necessary since there is only one state that uses the velocity, but it is left in to preserve the FSM (finite state machine) structure if more states are added in the future that use different outputs.
    switch (this.m_currentState) { 
      case FEEDING:
        // Single-motor feeder: preserve FSM structure while applying a single feed output.
        output = this.m_currentState.m_velocity;
        break;
      default:
        output = this.m_currentState.m_velocity;
        break;
    }

    this.m_io.setFeeder(output);

    this.m_io.update(m_currentState.getSubsystemPeriodicFrequency().in(Seconds)); //Update the IO's current frequency in seconds, which is used for dynamic scheduling of the subsystem's periodic method.
    Logger.recordOutput("Feeder/State", m_currentState);
    Logger.recordOutput("Feeder/Output", output);
  }

  /**
   * Enum representing the different states of the feeder.
   */
  public enum FEEDER_STATE {
    STOP(Milliseconds.of(60), 0),
    FEEDING(Milliseconds.of(20), 0.75);

    private final Time m_subsystemPeriodicFrequency;
    private final double m_velocity;

   /* Constructs a new FEEDER_STATE.
     *
     * @param pSubsystemPeriodicFrequency the frequency at which the subsystem should run in this state
     * @param pVelocity the velocity to set the feeder motor to in this state
     * 
     */
    FEEDER_STATE(Time pSubsystemPeriodicFrequency, double pVelocity) {
      this.m_subsystemPeriodicFrequency = pSubsystemPeriodicFrequency;
      this.m_velocity = pVelocity;
    }

    Time getSubsystemPeriodicFrequency() {
      return this.m_subsystemPeriodicFrequency;
    }

    double getVelocity() {
      return this.m_velocity;
    }
  }

  /**
   * Sets the current feeder state.
   *
   * @param state the feeder state to set
   */
  public void setState(FEEDER_STATE state) {
    if (m_testing) return;
    if (!this.m_currentState
        .getSubsystemPeriodicFrequency()
        .isEquivalent(state.getSubsystemPeriodicFrequency())) {
      m_timesConsumer.accept(Subsystems.Feeder, state.getSubsystemPeriodicFrequency());
    }
    this.m_currentState = state;
    Logger.recordOutput("Feeder/State", state);
  }

  /**
   * Sets the current feeder state for testing mode only.
   *
   * @param state the feeder state to set
   */
  public void setStateTesting(FEEDER_STATE state) {
    if (!m_testing) return;
    if (!this.m_currentState
        .getSubsystemPeriodicFrequency()
        .isEquivalent(state.getSubsystemPeriodicFrequency())) {
      m_timesConsumer.accept(Subsystems.Feeder, state.getSubsystemPeriodicFrequency());
    }
    this.m_currentState = state;
    Logger.recordOutput("Feeder/State", state);
  }

  /**
   * Enables or disables testing mode.
   *
   * @param testing true to enable testing mode
   */
  public void setTesting(boolean testing) {
    this.m_testing = testing;
  }

  /** 
  * Returns the current feeder state.
  * This method allows other parts of the code to access the current state of the feeder
  */
  public FEEDER_STATE getState() {
    return this.m_currentState;
  }
}
