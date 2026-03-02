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

@Logged
public class Intake {
  @Logged(importance = Importance.CRITICAL)
  private final IntakeIO m_io;

  @Logged(importance = Importance.CRITICAL)
  private boolean m_wasUp = false;

  @NotLogged private final TimesConsumer m_timesConsumer;

  @Logged(importance = Importance.INFO)
  private IntakeStates m_currentState;

  // Jam for rollers
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

  // Stuck for deployment
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

  /** Creates a new Intake. */
  public Intake(IntakeIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;

    this.m_currentState = IntakeStates.Up;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    m_io.update();

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
                  m_currentState.deploymentVelocity,
                  m_currentState.deploymentAcceleration);
            } else {
              m_io.setAngle(
                  IntakeStates.Down.setpoint,
                  IntakeStates.Down.deploymentVelocity,
                  IntakeStates.Down.deploymentAcceleration);
            }
          } else {
            m_stuckUndoTime.calculate(false);
            m_io.setAngle(
                m_currentState.setpoint,
                m_currentState.deploymentVelocity,
                m_currentState.deploymentAcceleration);
          }
        } else {
          m_stuckTime.calculate(false);
          m_stuckUndoTime.calculate(false);
          m_wasStuck = false;
          m_io.setAngle(
              m_currentState.setpoint,
              m_currentState.deploymentVelocity,
              m_currentState.deploymentAcceleration);
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
            m_wasUp ? m_currentState.setpoint : m_currentState.alternate,
            m_currentState.deploymentVelocity,
            m_currentState.deploymentAcceleration);
        break;
      default:
        m_stuckTime.calculate(false);
        m_minimumStuckTime.calculate(false);
        m_stuckUndoTime.calculate(false);
        m_wasStuck = false;
        m_io.setAngle(
            m_currentState.setpoint,
            m_currentState.deploymentVelocity,
            m_currentState.deploymentAcceleration);
        m_wasUp = false;
        break;
    }
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean isJammed() {
    return m_io.getRollerCurrent().in(Amps)
            >= JamDetectionConstants.IntakeRollers.kJamCurrent.in(Amps)
        && m_io.getRollerVelocity().in(RotationsPerSecond)
            <= JamDetectionConstants.IntakeRollers.kJamSpeedThreshold.in(RotationsPerSecond);
  }

  @Logged(importance = Importance.CRITICAL)
  public boolean isStuck() {
    return m_io.getDeploymentCurrent().in(Amps)
        >= JamDetectionConstants.DeploymentMotor.kStuckCurrent.in(Amps);
  }

  public void setState(IntakeStates state) {
    if (!m_currentState.subsystemPeriodicFrequency.isEquivalent(state.subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Intake, state.subsystemPeriodicFrequency);
    }
    m_currentState = state;
  }

  public enum IntakeStates {
    // TODO: Tune when we get deployment motor
    Up(Milliseconds.of(60), Rotations.of(0.29), 0, 1, 0.5),
    Down(Milliseconds.of(60), Degrees.of(0), 0, 1, 0.5),
    Jostle(Milliseconds.of(20), Rotations.of(0.05), Rotations.of(0.15), 0, 1, 0.5),
    Jammed(Milliseconds.of(20), Degrees.of(0), -0.3, 1, 0.5),
    Intaking(Milliseconds.of(20), Degrees.of(0), 1, 1, 0.5);

    private final Time subsystemPeriodicFrequency;
    final Angle setpoint;
    private final Angle alternate;
    private final double rollerSpeed;
    private final double deploymentVelocity;
    private final double deploymentAcceleration;

    IntakeStates(
        Time subsystemPeriodicFrequency,
        Angle angle,
        Angle alternate,
        double rollerSpeed,
        double deploymentVelocity,
        double deploymentAcceleration) {
      this.subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.setpoint = angle;
      this.alternate = alternate;
      this.rollerSpeed = rollerSpeed;
      this.deploymentVelocity = deploymentVelocity;
      this.deploymentAcceleration = deploymentAcceleration;
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
