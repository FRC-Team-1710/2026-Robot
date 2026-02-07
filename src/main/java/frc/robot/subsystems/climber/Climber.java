// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;

@Logged
public class Climber {
  /** Creates a new Climber. */
  private ClimberIO m_io;

  /*DEGREES*/
  private double m_wiggle = 1;
  private double speed = 0.5;

  private ClimberStates m_state = ClimberStates.DOWN;

  public Climber(ClimberIO io) {
    this.m_io = io;
  }

  public void periodic() {
    double m_currentPosition = m_io.getDegrees();
    double wantedAngle = m_state.m_angle;

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

  public enum ClimberStates {
    UP(0),
    DOWN(180);

    private final double m_angle;

    ClimberStates(double angle) {
      this.m_angle = angle;
    }
  }
}
