// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.MechanismUtil.ClimberVisualSim;

@Logged
public class ClimberIOSIM implements ClimberIO {
  private final DCMotor climberMotor;
  private final SingleJointedArmSim armPhysicsSim;
  private final ClimberVisualSim climberVisualSim;

  /** Creates a new ClimberIOSIM. */
  public ClimberIOSIM() {
    climberMotor = DCMotor.getKrakenX60(1);
    armPhysicsSim =
        new SingleJointedArmSim(climberMotor, 25, 0.004, 8, -45, 180, false, 45, new double[2]);
    climberVisualSim = new MechanismUtil().new ClimberVisualSim("Climber", .25, .125);
  }

  @Override
  public void setSpeed(double speed) {
    armPhysicsSim.setInput(speed);
    climberVisualSim.updateArm(getDegrees(), speed == 0);
  }

  public double getDegrees() {
    return Units.radiansToDegrees(armPhysicsSim.getAngleRads());
  }
}
