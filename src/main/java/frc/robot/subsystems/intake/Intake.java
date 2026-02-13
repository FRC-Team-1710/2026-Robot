// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.JamDetectionConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Intake {
  private final IntakeIO m_io;
  private final TimesConsumer m_timesConsumer;
  private IntakeStates m_currentState;

  private final Debouncer m_jamTime =
      new Debouncer(JamDetectionConstants.Intake.kJamMinimumTime.in(Seconds));
  private final Debouncer m_minimumJamTime =
      new Debouncer(JamDetectionConstants.Intake.kJamDetectionDisabledTime.in(Seconds));
  private final Debouncer m_jamUndoTime =
      new Debouncer(JamDetectionConstants.Intake.kJamUndoTime.in(Seconds));

  private boolean m_wasJammed = false;

  /** Creates a new Intake. */
  public Intake(IntakeIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;

    this.m_currentState = IntakeStates.Up;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    m_io.setAngle(m_currentState.setpoint, m_currentState.subsystemPeriodicFrequency.in(Seconds));
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
              m_io.setIntakeMotor(m_currentState.speed);
            } else {
              m_io.setIntakeMotor(IntakeStates.Jammed.speed);
            }
          } else {
            m_jamUndoTime.calculate(false);
            m_io.setIntakeMotor(m_currentState.speed);
          }
        } else {
          m_jamTime.calculate(false);
          m_jamUndoTime.calculate(false);
          m_wasJammed = false;
          m_io.setIntakeMotor(m_currentState.speed);
        }
        break;
      default:
        m_jamTime.calculate(false);
        m_minimumJamTime.calculate(false);
        m_jamUndoTime.calculate(false);
        m_wasJammed = false;
        m_io.setIntakeMotor(m_currentState.speed);
        break;
    }
  }

  public boolean isJammed() {
    return m_io.getRollerCurrent().in(Amps) >= JamDetectionConstants.Intake.kJamCurrent.in(Amps)
        && m_io.getRollerVelocity().in(RotationsPerSecond)
            <= JamDetectionConstants.Intake.kJamSpeedThreshold.in(RotationsPerSecond);
  }

  public void setState(IntakeStates state) {
    if (!m_currentState.subsystemPeriodicFrequency.isEquivalent(state.subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Intake, state.subsystemPeriodicFrequency);
    }
    m_currentState = state;
  }

  public enum IntakeStates {
    Up(Milliseconds.of(60), Degrees.of(90), 0),
    Down(Milliseconds.of(60), Degrees.of(-14.5), 0),
    Jammed(Milliseconds.of(20), Degrees.of(-14.5), -0.3),
    Intaking(Milliseconds.of(20), Degrees.of(0), .3);

    private final Time subsystemPeriodicFrequency;
    private final Angle setpoint;
    private final double speed;

    IntakeStates(Time subsystemPeriodicFrequency, Angle angle, double speed) {
      this.subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.setpoint = angle;
      this.speed = speed;
    }
  }
}
