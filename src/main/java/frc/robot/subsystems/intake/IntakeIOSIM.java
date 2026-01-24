// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.MechanismUtil;
import frc.robot.utils.MechanismUtil.IntakeVisualSim;

@Logged
public class IntakeIOSIM implements IntakeIO {
  /** Creates a new IntakeIOSIM. */
  private final DCMotor gearbox;

  private final SingleJointedArmSim armPhysicsSim;
  private final IntakeVisualSim intakeVisualSim;
  private Angle angleSetpoint;

  private final ProfiledPIDController PID = new ProfiledPIDController(
      5, 0, 0, new Constraints(400, 400));

  public IntakeIOSIM() {
    gearbox = DCMotor.getKrakenX60(1);
    armPhysicsSim =
        new SingleJointedArmSim(gearbox, 25, 0.004, 10, -45, 90, false, 45, new double[2]);

    intakeVisualSim = new MechanismUtil().new IntakeVisualSim("Intake", .25, .125); // creates the visual sim
  }

  public void setAngle(Angle angle) {
    angleSetpoint = angle;
    if (angleSetpoint == null) return;
    double armAngleRadians = armPhysicsSim.getAngleRads();
    double out = PID.calculate(armAngleRadians, angleSetpoint.in(Radians));
    Robot.telemetry().log("arm volts", out);
    armPhysicsSim.setInputVoltage(Math.max(-12.0, Math.min(12.0, out)));
    armPhysicsSim.update(0.02);
    intakeVisualSim.updateArm(
        Units.radiansToDegrees(armPhysicsSim.getAngleRads()),
        Math.abs(angleSetpoint.in(Radians) - armPhysicsSim.getAngleRads())
            < Units.degreesToRadians(1)); // updates visuals
    SmartDashboard.putData("ArmVisuals", intakeVisualSim.getMechanism());
  }

  public void setIntakeMotor(double speed) {
    intakeVisualSim.updateRoller(speed * 20); // updates roller visuals
    Robot.telemetry().log("RollerSpeed", speed);
  }
}
