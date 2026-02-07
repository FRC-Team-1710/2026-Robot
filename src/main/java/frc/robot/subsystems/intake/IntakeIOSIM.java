// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.MechanismUtil.IntakeVisualSim;

@Logged
public class IntakeIOSIM implements IntakeIO {
  /** Creates a new IntakeIOSIM. */
  private final DCMotor m_gearbox;

  private final SingleJointedArmSim m_armPhysicsSim;
  private final IntakeVisualSim m_intakeVisualSim;
  private Angle m_angleSetpoint;

  private final ProfiledPIDController m_PID =
      new ProfiledPIDController(5, 0, 0, new Constraints(400, 400));

  public IntakeIOSIM() {
    m_gearbox = DCMotor.getKrakenX60(1);
    m_armPhysicsSim =
        new SingleJointedArmSim(m_gearbox, 25, 0.004, 10, -45, 90, false, 45, new double[2]);

    m_intakeVisualSim =
        new MechanismUtil().new IntakeVisualSim("Intake", .25, .125); // creates the visual sim

    SmartDashboard.putNumber("Intake/JamTest/Velocity", 0);
    SmartDashboard.putNumber("Intake/JamTest/Current", 0);
  }

  public void setAngle(Angle angle) {
    m_angleSetpoint = angle;
    if (m_angleSetpoint == null) return;
    m_armPhysicsSim.setInputVoltage(
        m_PID.calculate(m_armPhysicsSim.getAngleRads(), m_angleSetpoint.in(Radians)));
    m_armPhysicsSim.update(0.02);
    m_intakeVisualSim.updateArm(
        Units.radiansToDegrees(m_armPhysicsSim.getAngleRads()),
        Math.abs(m_angleSetpoint.in(Radians) - m_armPhysicsSim.getAngleRads())
            < Units.degreesToRadians(1)); // updates visuals
    SmartDashboard.putData("ArmVisuals", m_intakeVisualSim.getMechanism());
  }

  public void setIntakeMotor(double speed) {
    m_intakeVisualSim.updateRoller(speed * 20); // updates roller visuals
    Robot.telemetry().log("RollerSpeed", speed);
  }

  public AngularVelocity getRollerVelocity() {
    return RotationsPerSecond.of(SmartDashboard.getNumber("Intake/JamTest/Velocity", 0));
  }

  public Current getRollerCurrent() {
    return Amps.of(SmartDashboard.getNumber("Intake/JamTest/Current", 0));
  }
}
