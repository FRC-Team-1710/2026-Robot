// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.MechanismUtil.ArmMechanism;

@Logged
public class IntakeIOSIM implements IntakeIO {
  /** Creates a new IntakeIOSIM. */
  private final DCMotor gearbox;

  private final SingleJointedArmSim armPhysicsSim;
  private final ArmMechanism armVisualSim;
  private Angle angleSetpoint;

  public IntakeIOSIM() {
    gearbox = DCMotor.getKrakenX60(1);
    armPhysicsSim =
        new SingleJointedArmSim(gearbox, 25, 0.004, 0.1, -45, 90, true, 0, new double[2]);
    armVisualSim = new ArmMechanism("Intake", 100); // creates the visual sim
  }

  public void setAngle(Angle angle) {
    angleSetpoint = angle;
    armPhysicsSim.setState(angle.magnitude(), Units.degreesToRadians(360));
    armVisualSim.update(
        Units.radiansToDegrees(armPhysicsSim.getAngleRads()),
        Math.abs(angleSetpoint.in(Radians) - armPhysicsSim.getAngleRads())
            < Units.degreesToRadians(1)); // updates visuals
  }
}
