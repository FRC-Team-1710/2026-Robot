// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Climber {
  /** Creates a new Climber. */
  private ClimberIO m_io;

  /*DEGREES*/
  private double m_wiggle = 1;
  private double speed = 0.5;

  private final TimesConsumer timesConsumer;

  private ClimberStates m_currentState = ClimberStates.DOWN;

  public Climber(ClimberIO io, TimesConsumer timesConsumer) {
    this.m_io = io;
    this.timesConsumer = timesConsumer;
  }

  public void periodic() {
    double m_currentPosition = m_io.getDegrees();
    double wantedAngle = m_currentState.m_angle.in(Degrees);

    if (wantedAngle - m_wiggle < m_currentPosition && m_currentPosition < wantedAngle + m_wiggle) {
      m_io.setSpeed(0);
    } else if (m_currentPosition < wantedAngle) {
      m_io.setSpeed(speed);
    } else if (m_currentPosition > wantedAngle) {
      m_io.setSpeed(-speed);
    } else {
      DriverStation.reportError("Climber is not in a valid state", false);
    }
  }

  public void setState(ClimberStates state) {
    if (!m_currentState.subsystemPeriodicFrequency.isEquivalent(state.subsystemPeriodicFrequency)) {
      timesConsumer.accept(Subsystems.Climber, state.subsystemPeriodicFrequency);
    }
    m_currentState = state;
  }

  public enum ClimberStates {
    UP(Milliseconds.of(60), Degrees.of(0)),
    DOWN(Milliseconds.of(20), Degrees.of(90));

    private final Time subsystemPeriodicFrequency;
    private final Angle m_angle;

    ClimberStates(Time subsystemPeriodicFrequency, Angle angle) {
      this.m_angle = angle;
      this.subsystemPeriodicFrequency = subsystemPeriodicFrequency;
    }
  }
}
