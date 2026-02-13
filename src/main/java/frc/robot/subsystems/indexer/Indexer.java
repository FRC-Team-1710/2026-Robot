// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.JamDetectionConstants;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Indexer {
  private final IndexerIO io;
  private final TimesConsumer timesConsumer;
  private IndexStates currentState = IndexStates.Idle;
  private final Debouncer jamTime =
      new Debouncer(JamDetectionConstants.Intake.jamMinimumTime.in(Seconds));
  private final Debouncer minimumJamTime =
      new Debouncer(JamDetectionConstants.Intake.jamDetectionDisabledTime.in(Seconds));
  private final Debouncer jamUndoTime =
      new Debouncer(JamDetectionConstants.Intake.jamUndoTime.in(Seconds));

  private boolean wasJammed = false;

  /** Creates a new Index. */
  public Indexer(IndexerIO io, TimesConsumer consumer) {
    this.io = io;
    this.timesConsumer = consumer;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    io.setIndexMotor(currentState.speed);
    io.updateVisual();
    switch (currentState) {
      case Indexing:
        // IMPORTANT, keep every if statement different!
        if (minimumJamTime.calculate(true)) {
          if (jamTime.calculate(isJammed()) || wasJammed) {
            wasJammed = true;
            if (jamUndoTime.calculate(true)) {
              jamTime.calculate(false);
              jamUndoTime.calculate(false);
              wasJammed = false;
              io.setIndexMotor(currentState.speed);
            } else {
              io.setIndexMotor(IndexStates.Jammed.speed);
            }
          } else {
            jamUndoTime.calculate(false);
            io.setIndexMotor(currentState.speed);
          }
        } else {
          jamTime.calculate(false);
          jamUndoTime.calculate(false);
          wasJammed = false;
          io.setIndexMotor(currentState.speed);
        }
        break;
      default:
        jamTime.calculate(false);
        minimumJamTime.calculate(false);
        jamUndoTime.calculate(false);
        wasJammed = false;
        io.setIndexMotor(currentState.speed);
        break;
    }
  }

  public boolean isJammed() {
    return io.getIndexMotorCurrent().in(Amps) >= JamDetectionConstants.Intake.jamCurrent.in(Amps)
        && io.getIndexMotorVelocity().in(RotationsPerSecond)
            <= JamDetectionConstants.Intake.JamSpeedThreshold.in(RotationsPerSecond);
  }

  public void setState(IndexStates state) {
    if (!currentState.subsystemPeriodicFrequency.isEquivalent(state.subsystemPeriodicFrequency)) {
      timesConsumer.accept(Subsystems.Indexer, state.subsystemPeriodicFrequency);
    }
    currentState = state;
  }

  public enum IndexStates {
    Indexing(Milliseconds.of(20), 0.5),

    Idle(Milliseconds.of(20), 0.5),

    Jammed(Milliseconds.of(60), 0);

    private final Time subsystemPeriodicFrequency;
    private final double speed;

    IndexStates(Time subsystemPeriodicFrequency, double speed) {
      this.subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.speed = speed;
    }
  }
}
