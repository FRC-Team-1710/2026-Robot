// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

public class Indexer {
  private final IndexerIO io;
  private IndexStates currentState = IndexStates.Idle;

  /** Creates a new Index. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    io.setIndexMotor(currentState.speed);
  }

  public void setState(IndexStates state) {
    currentState = state;
  }

  public enum IndexStates {
    Indexing(0),

    Idle(0);

    private final double speed;

    IndexStates(double speed) {
      this.speed = speed;
    }
  }
}
