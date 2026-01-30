// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Indexer {
  private final IndexerIO io;
  private final TimesConsumer timesConsumer;
  private IndexStates currentState = IndexStates.Idle;

  /** Creates a new Index. */
  public Indexer(IndexerIO io, TimesConsumer consumer) {
    this.io = io;
    this.timesConsumer = consumer;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    io.setIndexMotor(currentState.speed);
  }

  public void setState(IndexStates state) {
    if (!currentState.subsystemPeriodicFrequency.isEquivalent(state.subsystemPeriodicFrequency)) {
      timesConsumer.accept(Subsystems.Indexer, state.subsystemPeriodicFrequency);
      // Don't delete!
      periodic();
    }
    currentState = state;
  }

  public enum IndexStates {
    Indexing(Milliseconds.of(20), 0),

    Idle(Milliseconds.of(60), 0);

    private final Time subsystemPeriodicFrequency;
    private final double speed;

    IndexStates(Time subsystemPeriodicFrequency, double speed) {
      this.subsystemPeriodicFrequency = subsystemPeriodicFrequency;
      this.speed = speed;
    }
  }
}
