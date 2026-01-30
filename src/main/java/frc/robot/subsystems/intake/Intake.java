// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Intake {
  private final IntakeIO m_io;
  private final TimesConsumer m_timesConsumer;
  private IntakeStates m_currentState;

  /** Creates a new Intake. */
  public Intake(IntakeIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;

    this.m_currentState = IntakeStates.Up;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    m_io.setAngle(m_currentState.m_setpoint);
    m_io.setIntakeMotor(m_currentState.m_speed);
  }

  public void setState(IntakeStates state) {
    if (!m_currentState.m_subsystemPeriodicFrequency.isEquivalent(
        state.m_subsystemPeriodicFrequency)) {
      m_timesConsumer.accept(Subsystems.Intake, state.m_subsystemPeriodicFrequency);
    }
    m_currentState = state;
  }

  public enum IntakeStates {
    Up(Milliseconds.of(60), Degrees.of(90), 0),
    Down(Milliseconds.of(60), Degrees.of(-14.5), 0),
    Intaking(Milliseconds.of(20), Degrees.of(0), .3);

    private final Time m_subsystemPeriodicFrequency;
    private final Angle m_setpoint;
    private final double m_speed;

    IntakeStates(Time subsystemPeriodicFrequency, Angle angle, double speed) {
      this.m_subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.m_setpoint = angle;
      this.m_speed = speed;
    }
  }
}
