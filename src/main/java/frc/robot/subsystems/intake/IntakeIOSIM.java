// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.MechanismUtil.ArmMechanism;

@Logged
public class IntakeIOSIM implements IntakeIO {
  /** Creates a new IntakeIOSIM. */
  private final SingleJointedArmSim armPhysicsSim;
  private final ArmMechanism armVisualSim;
  private Angle angleSetpoint;

  public IntakeIOSIM() {
    armPhysicsSim = new SingleJointedArmSim(null, 1, 0, 0, 0, 0, true, 0, null);
    armVisualSim = new ArmMechanism("Intake", 100); //creates the visual sim
  }
  
  public void setAngle(Angle angle) {
    angleSetpoint = angle;
    armPhysicsSim.setState(angle.magnitude(), Units.degreesToRadians(360));
    armVisualSim.update(Units.radiansToDegrees(armPhysicsSim.getAngleRads()), Math.abs(angleSetpoint.in(Radians) - armPhysicsSim.getAngleRads()) < Units.degreesToRadians(1) ); //updates visuals
  }
}
