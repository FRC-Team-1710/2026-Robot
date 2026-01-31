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
  private final IntakeIO io;
  private final TimesConsumer timesConsumer;
  private IntakeStates currentState;

  private final Debouncer jamTime =
      new Debouncer(JamDetectionConstants.Intake.jamMinimumTime.in(Seconds));
  private final Debouncer minimumJamTime =
      new Debouncer(JamDetectionConstants.Intake.jamDetectionDisabledTime.in(Seconds));
  private final Debouncer jamUndoTime =
      new Debouncer(JamDetectionConstants.Intake.jamUndoTime.in(Seconds));

  private boolean wasJammed = false;

  /** Creates a new Intake. */
  public Intake(IntakeIO io, TimesConsumer consumer) {
    this.io = io;
    this.timesConsumer = consumer;

    this.currentState = IntakeStates.Up;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    io.setAngle(currentState.setpoint);
    switch (currentState) {
      case Intaking:
        // IMPORTANT, keep every if statement different!
        if (minimumJamTime.calculate(true)) {
          if (jamTime.calculate(isJammed()) || wasJammed) {
            wasJammed = true;
            if (jamUndoTime.calculate(true)) {
              jamTime.calculate(false);
              jamUndoTime.calculate(false);
              wasJammed = false;
              io.setIntakeMotor(currentState.speed);
            } else {
              io.setIntakeMotor(IntakeStates.Jammed.speed);
            }
          } else {
            jamUndoTime.calculate(false);
            io.setIntakeMotor(currentState.speed);
          }
        } else {
          jamTime.calculate(false);
          jamUndoTime.calculate(false);
          wasJammed = false;
          io.setIntakeMotor(currentState.speed);
        }
        break;
      default:
        jamTime.calculate(false);
        minimumJamTime.calculate(false);
        jamUndoTime.calculate(false);
        wasJammed = false;
        io.setIntakeMotor(currentState.speed);
        break;
    }
  }

  public boolean isJammed() {
    return io.getRollerCurrent().in(Amps) >= JamDetectionConstants.Intake.jamCurrent.in(Amps)
        && io.getRollerVelocity().in(RotationsPerSecond)
            <= JamDetectionConstants.Intake.jamSpeedThreshold.in(RotationsPerSecond);
  }

  public void setState(IntakeStates state) {
    if (!currentState.subsystemPeriodicFrequency.isEquivalent(state.subsystemPeriodicFrequency)) {
      timesConsumer.accept(Subsystems.Intake, state.subsystemPeriodicFrequency);
    }
    currentState = state;
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
