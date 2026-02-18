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
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.JamDetectionConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Intake {
  @Logged(importance = Importance.DEBUG)
  private final IntakeIO m_io;

  @Logged(importance = Importance.DEBUG)
  private final TimesConsumer m_timesConsumer;

  @Logged(importance = Importance.DEBUG)
  private IntakeStates m_currentState;

  // Jam for rollers
  @Logged(importance = Importance.DEBUG)
  private final Debouncer m_jamTime =
      new Debouncer(JamDetectionConstants.IntakeRollers.kJamMinimumTime.in(Seconds));

  @Logged(importance = Importance.DEBUG)
  private final Debouncer m_minimumJamTime =
      new Debouncer(JamDetectionConstants.IntakeRollers.kJamDetectionDisabledTime.in(Seconds));

  @Logged(importance = Importance.DEBUG)
  private final Debouncer m_jamUndoTime =
      new Debouncer(JamDetectionConstants.IntakeRollers.kJamUndoTime.in(Seconds));

  @Logged(importance = Importance.DEBUG)
  private boolean m_wasJammed = false;

  // Stuck for deployment
  @Logged(importance = Importance.DEBUG)
  private final Debouncer m_stuckTime =
      new Debouncer(JamDetectionConstants.DeploymentMotor.kStuckMinimumTime.in(Seconds));

  @Logged(importance = Importance.DEBUG)
  private final Debouncer m_minimumStuckTime =
      new Debouncer(JamDetectionConstants.DeploymentMotor.kStuckDetectionDisabledTime.in(Seconds));

  @Logged(importance = Importance.DEBUG)
  private final Debouncer m_stuckUndoTime =
      new Debouncer(JamDetectionConstants.DeploymentMotor.kStuckUndoTime.in(Seconds));

  @Logged(importance = Importance.DEBUG)
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
              m_io.setIntakeMotor(
                  m_currentState.rollerSpeed,
                  m_currentState.deploymentVelocity,
                  m_currentState.deploymentAcceleration);
            } else {
              m_io.setIntakeMotor(
                  IntakeStates.Jammed.rollerSpeed,
                  IntakeStates.Jammed.deploymentVelocity,
                  IntakeStates.Jammed.deploymentAcceleration);
            }
          } else {
            m_jamUndoTime.calculate(false);
            m_io.setIntakeMotor(
                m_currentState.rollerSpeed,
                m_currentState.deploymentVelocity,
                m_currentState.deploymentAcceleration);
          }
        } else {
          m_jamTime.calculate(false);
          m_jamUndoTime.calculate(false);
          m_wasJammed = false;
          m_io.setIntakeMotor(
              m_currentState.rollerSpeed,
              m_currentState.deploymentVelocity,
              m_currentState.deploymentAcceleration);
        }
        break;
      default:
        m_jamTime.calculate(false);
        m_minimumJamTime.calculate(false);
        m_jamUndoTime.calculate(false);
        m_wasJammed = false;
        m_io.setIntakeMotor(
            m_currentState.rollerSpeed,
            m_currentState.deploymentVelocity,
            m_currentState.deploymentAcceleration);
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
              m_io.setAngle(m_currentState.setpoint);
            } else {
              m_io.setAngle(IntakeStates.Down.setpoint);
            }
          } else {
            m_stuckUndoTime.calculate(false);
            m_io.setAngle(m_currentState.setpoint);
          }
        } else {
          m_stuckTime.calculate(false);
          m_stuckUndoTime.calculate(false);
          m_wasStuck = false;
          m_io.setAngle(m_currentState.setpoint);
        }
        break;
      default:
        m_stuckTime.calculate(false);
        m_minimumStuckTime.calculate(false);
        m_stuckUndoTime.calculate(false);
        m_wasStuck = false;
        m_io.setAngle(m_currentState.setpoint);
        break;
    }
  }

  @Logged(importance = Importance.DEBUG)
  public boolean isJammed() {
    return m_io.getRollerCurrent().in(Amps)
            >= JamDetectionConstants.IntakeRollers.kJamCurrent.in(Amps)
        && m_io.getRollerVelocity().in(RotationsPerSecond)
            <= JamDetectionConstants.IntakeRollers.kJamSpeedThreshold.in(RotationsPerSecond);
  }

  @Logged(importance = Importance.DEBUG)
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
    Up(Milliseconds.of(60), Degrees.of(90), 0, 50, 50),
    Down(Milliseconds.of(60), Degrees.of(-14.5), 0, 100, 100),
    Jammed(Milliseconds.of(20), Degrees.of(-14.5), -0.3, 100, 100),
    Intaking(Milliseconds.of(20), Degrees.of(0), 1, 100, 100);

    private final Time subsystemPeriodicFrequency;
    private final Angle setpoint;
    private final double rollerSpeed;
    private final double deploymentVelocity;
    private final double deploymentAcceleration;

    IntakeStates(
        Time subsystemPeriodicFrequency,
        Angle angle,
        double rollerSpeed,
        double deploymentVelocity,
        double deploymentAcceleration) {
      this.subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.setpoint = angle;
      this.rollerSpeed = rollerSpeed;
      this.deploymentVelocity = deploymentVelocity;
      this.deploymentAcceleration = deploymentAcceleration;
    }
  }
}
