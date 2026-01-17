// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;

@Logged
public class Intake {
  private final IntakeIO io;
  private IntakeStates currentState;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    this.currentState = IntakeStates.Up;
  }

  public void periodic() {
    // This method will be called once per scheduler run
    io.setAngle(currentState.setpoint);
    io.setIntakeMotor(currentState.speed);
  }

  public void setState(IntakeStates state) {
    currentState = state;
  }

  public enum IntakeStates {
    Up(Degrees.of(0), 0),
    Down(Degrees.of(0), 0),
    Intaking(Degrees.of(0), 0);

    private final Angle setpoint;
    private final double speed;

    IntakeStates(Angle angle, double speed) {
      this.setpoint = angle;
      this.speed = speed;
    }
  }
}
