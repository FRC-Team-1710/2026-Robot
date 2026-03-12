// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.JamDetectionConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import java.util.function.BooleanSupplier;

/**
 * The Intake subsystem coordinates the intake mechanism: roller control and deployment (arm)
 * positioning. It contains the high-level state machine for intaking, jam detection and recovery,
 * and deployment "stuck" detection.
 *
 * <p>Responsibilities:
 *
 * <ul>
 *   <li>Manage desired roller speed and deployment setpoint based on {@link Intake.IntakeStates}.
 *   <li>Detect roller jams and apply a recovery behavior when necessary.
 *   <li>Detect deployment motor stuck conditions and protect the mechanism.
 * </ul>
 *
 * <p>This class delegates hardware access to an {@link IntakeIO} implementation (CTRE hardware or a
 * simulation). All public-facing behaviors are expressed through {@link #setState(IntakeStates)}
 * and the periodic update loop.
 */
@Logged
public class Intake {
  /** IO implementation used to read sensors and command motors (hardware or simulation). */
  @Logged(importance = Importance.CRITICAL)
  private final IntakeIO m_io;

  /** Boolean representing if the intake was up during jostling. */
  @Logged(importance = Importance.CRITICAL)
  private boolean m_wasUp = false;

  @NotLogged private boolean m_testing = false;

  /** Callback used to request changes to the subsystem periodic frequency. */
  @NotLogged private final TimesConsumer m_timesConsumer;

  @Logged(importance = Importance.INFO)
  private IntakeStates m_currentState;

  @NotLogged
  private final Debouncer m_jamTime =
      new Debouncer(JamDetectionConstants.IntakeRollers.kJamMinimumTime.in(Seconds));

  @NotLogged
  private final Debouncer m_minimumJamTime =
      new Debouncer(JamDetectionConstants.IntakeRollers.kJamDetectionDisabledTime.in(Seconds));

  @NotLogged
  private final Debouncer m_jamUndoTime =
      new Debouncer(JamDetectionConstants.IntakeRollers.kJamUndoTime.in(Seconds));

  @Logged(importance = Importance.INFO)
  private boolean m_wasJammed = false;

  @NotLogged
  private final Debouncer m_stuckTime =
      new Debouncer(JamDetectionConstants.DeploymentMotor.kStuckMinimumTime.in(Seconds));

  @NotLogged
  private final Debouncer m_minimumStuckTime =
      new Debouncer(JamDetectionConstants.DeploymentMotor.kStuckDetectionDisabledTime.in(Seconds));

  @NotLogged
  private final Debouncer m_stuckUndoTime =
      new Debouncer(JamDetectionConstants.DeploymentMotor.kStuckUndoTime.in(Seconds));

  @Logged(importance = Importance.INFO)
  private boolean m_wasStuck = false;

  @NotLogged private final BooleanSupplier m_bumpSupplier;

  /**
   * Constructs an Intake subsystem.
   *
   * @param io the hardware/simulation IO implementation used to read sensors and command motors
   * @param consumer callback used to request subsystem periodic frequency changes
   */
  public Intake(IntakeIO io, TimesConsumer consumer, BooleanSupplier bumpSupplier) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_bumpSupplier = bumpSupplier;

    this.m_currentState = IntakeStates.Up;
  }

  /**
   * Periodic update for the intake subsystem. Should be invoked by the robot framework once per
   * scheduler run. This updates IO, runs jam/stuck detection logic, and commands the IO layer with
   * the appropriate motor outputs or angle setpoints based on the current state.
   */
  public void periodic() {
    // This method will be called once per scheduler run
    m_io.update();
    if (m_bumpSupplier.getAsBoolean()) {
      m_io.setIntakeMotor(-IntakeStates.Intaking.rollerSpeed);
    } else {
      // Set the rollers with jam logic
      switch (m_currentState) {
        case Intaking:
          // IMPORTANT, keep every if statement different!
          if (m_minimumJamTime.calculate(true)) {
            if (m_jamTime.calculate(isJammed()) || m_wasJammed) {
              m_wasJammed = true;
              if (m_jamUndoTime.calculate(true)) {
                m_jamTime.calculate(false);
                m_jamUndoTime.calculate(false);
                m_wasJammed = false;
                m_io.setIntakeMotor(m_currentState.rollerSpeed);
              } else {
                m_io.setIntakeMotor(IntakeStates.Jammed.rollerSpeed);
              }
            } else {
              m_jamUndoTime.calculate(false);
              m_io.setIntakeMotor(m_currentState.rollerSpeed);
            }
          } else {
            m_jamTime.calculate(false);
            m_jamUndoTime.calculate(false);
            m_wasJammed = false;
            m_io.setIntakeMotor(m_currentState.rollerSpeed);
          }
          break;
        default:
          m_jamTime.calculate(false);
          m_minimumJamTime.calculate(false);
          m_jamUndoTime.calculate(false);
          m_wasJammed = false;
          m_io.setIntakeMotor(m_currentState.rollerSpeed);
          break;
      }
    }

    switch (m_currentState) {
      case Up:
        // IMPORTANT, keep every if statement different!
        if (m_minimumStuckTime.calculate(true)) {
          if (m_stuckTime.calculate(isStuck()) || m_wasStuck) {
            m_wasStuck = true;
            if (m_stuckUndoTime.calculate(true)) {
              m_stuckTime.calculate(false);
              m_stuckUndoTime.calculate(false);
              m_wasStuck = false;
              m_io.setAngle(
                  m_currentState.setpoint,
                  m_currentState.m_deploymentVelocity,
                  m_currentState.m_deploymentAcceleration);
            } else {
              m_io.setAngle(
                  IntakeStates.Down.setpoint,
                  IntakeStates.Down.m_deploymentVelocity,
                  IntakeStates.Down.m_deploymentAcceleration);
            }
          } else {
            m_stuckUndoTime.calculate(false);
            m_io.setAngle(
                m_currentState.setpoint,
                m_currentState.m_deploymentVelocity,
                m_currentState.m_deploymentAcceleration);
          }
        } else {
          m_stuckTime.calculate(false);
          m_stuckUndoTime.calculate(false);
          m_wasStuck = false;
          m_io.setAngle(
              m_currentState.setpoint,
              m_currentState.m_deploymentVelocity,
              m_currentState.m_deploymentAcceleration);
        }
        m_wasUp = false;
        break;
      case Jostle:
        m_stuckTime.calculate(false);
        m_minimumStuckTime.calculate(false);
        m_stuckUndoTime.calculate(false);
        m_wasStuck = false;
        if (m_io.getSetpointReferenceVelocityIsZero()) {
          m_wasUp = !m_wasUp;
        }
        m_io.setAngle(
            m_wasUp ? m_currentState.setpoint : m_currentState.m_alternate,
            m_currentState.m_deploymentVelocity,
            m_currentState.m_deploymentAcceleration);
        break;
      default:
        m_stuckTime.calculate(false);
        m_minimumStuckTime.calculate(false);
        m_stuckUndoTime.calculate(false);
        m_wasStuck = false;
        m_io.setAngle(
            m_currentState.setpoint,
            m_currentState.m_deploymentVelocity,
            m_currentState.m_deploymentAcceleration);
        m_wasUp = false;
        break;
    }
  }

  /**
   * Returns true when the intake rollers meet the configured jam criteria (high current and low
   * velocity).
   *
   * @return true if a roller jam is detected
   */
  @Logged(importance = Importance.CRITICAL)
  public boolean isJammed() {
    return m_io.getRollerCurrent().in(Amps)
            >= JamDetectionConstants.IntakeRollers.kJamCurrent.in(Amps)
        && m_io.getRollerVelocity().in(RotationsPerSecond)
            <= JamDetectionConstants.IntakeRollers.kJamSpeedThreshold.in(RotationsPerSecond);
  }

  /**
   * Returns true when the deployment motor current exceeds the configured stuck threshold.
   *
   * @return true if the deployment motor is considered stuck
   */
  @Logged(importance = Importance.CRITICAL)
  public boolean isStuck() {
    return m_io.getDeploymentCurrent().in(Amps)
        >= JamDetectionConstants.DeploymentMotor.kStuckCurrent.in(Amps);
  }

  /**
   * Request a transition to the given intake state. If the requested state's periodic frequency
   * differs from the current state's, the subsystem will notify the {@code TimesConsumer} so the
   * robot scheduler can adjust.
   *
   * @param state the desired {@link IntakeStates} to move into
   */
  public void setState(IntakeStates state) {
    if (m_testing) return;
    if (!m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        state.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Intake, state.m_subsystemPeriodicFrequency);
    }
    m_currentState = state;
  }

  /**
   * Sets the current intake state for testing mode only.
   *
   * @param state the intake state to set
   */
  public void setStateTesting(IntakeStates state) {
    if (!m_testing) return;
    if (!m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        state.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Intake, state.m_subsystemPeriodicFrequency);
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

  /** Returns the current intake state. */
  public IntakeStates getState() {
    return m_currentState;
  }

  public enum IntakeStates {
    Up(Milliseconds.of(60), Rotations.of(0.29), 0, 1, 0.5),
    Half(Milliseconds.of(60), Rotations.of(0.145), 0, 1, 0.5),
    Down(Milliseconds.of(60), Degrees.of(0), 0, 1, 0.5),
    Jostle(Milliseconds.of(20), Rotations.of(0.05), Rotations.of(0.125), 1, 1.5, 1),
    Jammed(Milliseconds.of(20), Degrees.of(0), -0.3, 1, 0.5),
    Intaking(Milliseconds.of(20), Degrees.of(0), 1, 1, 0.5);

    private final Time m_subsystemPeriodicFrequency;
    final Angle setpoint;
    private final Angle m_alternate;
    private final double rollerSpeed;
    private final double m_deploymentVelocity;
    private final double m_deploymentAcceleration;

    IntakeStates(
        Time subsystemPeriodicFrequency,
        Angle angle,
        Angle alternate,
        double rollerSpeed,
        double deploymentVelocity,
        double deploymentAcceleration) {
      this.m_subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.setpoint = angle;
      this.m_alternate = alternate;
      this.rollerSpeed = rollerSpeed;
      this.m_deploymentVelocity = deploymentVelocity;
      this.m_deploymentAcceleration = deploymentAcceleration;
    }

    IntakeStates(
        Time subsystemPeriodicFrequency,
        Angle angle,
        double rollerSpeed,
        double deploymentVelocity,
        double deploymentAcceleration) {
      this(
          subsystemPeriodicFrequency,
          angle,
          angle,
          rollerSpeed,
          deploymentVelocity,
          deploymentAcceleration);
    }
  }
}
