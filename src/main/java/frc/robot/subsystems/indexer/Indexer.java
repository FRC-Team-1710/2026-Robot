// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.JamDetectionConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Indexer {
  @Logged(importance = Logged.Importance.CRITICAL)
  private final IndexerIO m_io;

  @NotLogged private final TimesConsumer m_timesConsumer;

  @Logged(importance = Logged.Importance.CRITICAL)
  private IndexStates m_currentState = IndexStates.Idle;

  @NotLogged private boolean m_testing = false;

  @NotLogged
  private final Debouncer m_jamTime =
      new Debouncer(JamDetectionConstants.Indexer.kJamMinimumTime.in(Seconds));

  @NotLogged
  private final Debouncer m_minimumJamTime =
      new Debouncer(JamDetectionConstants.Indexer.kJamDetectionDisabledTime.in(Seconds));

  @NotLogged
  private final Debouncer m_jamUndoTime =
      new Debouncer(JamDetectionConstants.Indexer.kJamUndoTime.in(Seconds));

  @Logged(importance = Logged.Importance.INFO)
  private boolean m_wasJammed = false;

  /**
   * Creates a new Indexer.
   *
   * @param io the indexer IO implementation
   * @param consumer the times consumer for dynamic scheduling
   */
  public Indexer(IndexerIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
  }

  /** Runs periodic indexer logic including jam detection. */
  public void periodic() {
    // This method will be called once per scheduler run
    m_io.update();
    m_io.setIndexMotor(m_currentState.m_speed);
    if (m_bumpSupplier.getAsBoolean()) {
      // This is NOT a magic number, it makes it go backwards to hopefully unjam the Fuel
      m_io.setIndexMotor(-0.5);
    } else {
      switch (m_currentState) {
        case Indexing:
          // IMPORTANT, keep every if statement different!
          if (m_minimumJamTime.calculate(true)) {
            if (m_jamTime.calculate(isJammed()) || m_wasJammed) {
              m_wasJammed = true;
              if (m_jamUndoTime.calculate(true)) {
                m_jamTime.calculate(false);
                m_jamUndoTime.calculate(false);
                m_wasJammed = false;
                m_io.setIndexMotor(m_currentState.m_speed);
              } else {
                m_io.setIndexMotor(IndexStates.Jammed.m_speed);
              }
            } else {
              m_jamUndoTime.calculate(false);
              m_io.setIndexMotor(m_currentState.m_speed);
            }
          } else {
            m_jamTime.calculate(false);
            m_jamUndoTime.calculate(false);
            m_wasJammed = false;
            m_io.setIndexMotor(m_currentState.m_speed);
          }
          break;
        default:
          m_jamTime.calculate(false);
          m_minimumJamTime.calculate(false);
          m_jamUndoTime.calculate(false);
          m_wasJammed = false;
          m_io.setIndexMotor(m_currentState.m_speed);
          break;
      }
    io.update();
    io.setIndexMotor(currentState.speed);
    // This is NOT a magic number, it makes it go backwards to hopefully unjam the Fuel
    switch (currentState) {
      case Indexing:
        // IMPORTANT, keep every if statement different!
        if (minimumJamTime.calculate(true)) {
          if (m_jamTime.calculate(isJammed()) || wasJammed) {
            wasJammed = true;
            if (m_jamUndoTime.calculate(true)) {
              m_jamTime.calculate(false);
              m_jamUndoTime.calculate(false);
              wasJammed = false;
              io.setIndexMotor(currentState.speed);
            } else {
              io.setIndexMotor(IndexStates.Jammed.speed);
            }
          } else {
            m_jamUndoTime.calculate(false);
            io.setIndexMotor(currentState.speed);
          }
        } else {
          m_jamTime.calculate(false);
          m_jamUndoTime.calculate(false);
          wasJammed = false;
          io.setIndexMotor(currentState.speed);
        }
        break;
      default:
        m_jamTime.calculate(false);
        minimumJamTime.calculate(false);
        m_jamUndoTime.calculate(false);
        wasJammed = false;
        io.setIndexMotor(currentState.speed);
        break;
    }
  }

  /** Returns whether the indexer motor is jammed. */
  @Logged(importance = Logged.Importance.INFO)
  public boolean isJammed() {
    return m_io.getIndexMotorCurrent().in(Amps)
            >= JamDetectionConstants.Indexer.kJamCurrent.in(Amps)
        && m_io.getIndexMotorVelocity().in(RotationsPerSecond)
            <= JamDetectionConstants.Indexer.kJamSpeedThreshold.in(RotationsPerSecond);
  }

  /**
   * Sets the current indexer state.
   *
   * @param state the indexer state to set
   */
  public void setState(IndexStates state) {
    if (m_testing) return;
    if (!m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        state.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Indexer, state.m_subsystemPeriodicFrequency);
    }
    m_currentState = state;
  }

  /**
   * Sets the current indexer state for testing mode only.
   *
   * @param state the indexer state to set
   */
  public void setStateTesting(IndexStates state) {
    if (!m_testing) return;
    if (!m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        state.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Indexer, state.m_subsystemPeriodicFrequency);
    }
    m_currentState = state;
  }

  /**
   * Enables or disables testing mode.
   *
   * @param testing true to enable testing mode
   */
  public void setTesting(boolean testing) {
    m_testing = testing;
  }

  public enum IndexStates {
    Indexing(Milliseconds.of(20), 0.35),

    Idle(Milliseconds.of(20), 0),

    Jammed(Milliseconds.of(60), 0);

    private final Time m_subsystemPeriodicFrequency;
    private final double m_speed;

    IndexStates(Time subsystemPeriodicFrequency, double speed) {
      this.m_subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.m_speed = speed;
    }
  }
}
