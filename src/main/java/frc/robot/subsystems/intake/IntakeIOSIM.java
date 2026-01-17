// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import edu.wpi.first.epilogue.Logged;
import frc.robot.utils.MechanismUtil.ArmMechanism;

@Logged
public class IntakeIOSIM implements IntakeIO {
  /** Creates a new IntakeIOSIM. */
  ArmMechanism armMechanism;

  public IntakeIOSIM() {
    armMechanism = new ArmMechanism("Intake", 100); //creates the visual sim
  }

  public void periodic() {
    // This method will be called once per scheduler run
    armMechanism.update(0, false); //updates visuals
  }
}
