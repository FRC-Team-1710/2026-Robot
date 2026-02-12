// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.MechanismUtil.ClimberVisualSim;

@Logged
public class ClimberIOSIM implements ClimberIO {
  private final DCMotor m_climberMotor;
  private final SingleJointedArmSim m_armPhysicsSim;
  private final ClimberVisualSim m_climberVisualSim;

  /** Creates a new ClimberIOSIM. */
  public ClimberIOSIM() {
    m_climberMotor = DCMotor.getKrakenX60(1);
    m_armPhysicsSim =
        new SingleJointedArmSim(m_climberMotor, 25, 0.004, 1, -180, 180, false, 0, new double[2]);
    m_climberVisualSim = new MechanismUtil().new ClimberVisualSim("Climber", .25);
  }

  @Override
  public void setSpeed(double speed) {
    m_armPhysicsSim.setInput(speed);
    m_climberVisualSim.updateArm(getDegrees(), speed == 0);
    SmartDashboard.putData("ClimberVisuals", m_climberVisualSim.getMechanism());
  }

  public double getDegrees() {
    return Units.radiansToDegrees(m_armPhysicsSim.getAngleRads());
  }
}
