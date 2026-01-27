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
  private final IntakeIO io;
  private final TimesConsumer timesConsumer;
  private IntakeStates currentState;

  /** Creates a new Intake. */
  public Intake(IntakeIO io, TimesConsumer consumer) {
    this.io = io;
    this.timesConsumer = consumer;

    this.currentState = IntakeStates.Up;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    io.setAngle(currentState.setpoint);
    io.setIntakeMotor(currentState.speed);
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
