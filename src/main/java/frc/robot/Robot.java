// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Alliance;
import frc.robot.constants.MatchState;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.utils.DynamicTimedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends DynamicTimedRobot {

  private Command m_autonomousCommand;

  private boolean m_wasAuto = false;

  private final RobotContainer m_robotContainer;

  private final PowerDistribution pdhLogging = new PowerDistribution();

  private boolean m_hasAppliedTestingControls = false;

  public Robot() {
    Alliance.updateRedAlliance();

    m_robotContainer = new RobotContainer(this::setSubsystemConsumer);

    DataLogManager.start();

    DriverStation.silenceJoystickConnectionWarning(true);

    Logger.recordMetadata("ProjectName", "2026-Robot");
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(
        new WPILOGWriter(Mode.currentMode == CurrentMode.SIMULATION ? "logs" : "/media/sda1"));
    Logger.start();

    addAllSubsystems(m_robotContainer.getAllSubsystems());

    if (Mode.currentMode == CurrentMode.SIMULATION) {
      SmartDashboard.putBoolean("Reset Fuel Sim", false);
    }

    SmartDashboard.putBoolean("MatchState/IgnoreFMS", false);

    // Lowers brownout threshold to 6.0V
    RobotController.setBrownoutVoltage(6.0);

    DriverStation.silenceJoystickConnectionWarning(true);

    SignalLogger.stop();
    SignalLogger.setPath("/media/sda1");
    SignalLogger.start();

    m_wasAuto = false;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    MatchState.updateAutonomousWinner();
    Logger.recordOutput("Robot/PDHVoltage", pdhLogging.getVoltage());
  }

  @Override
  public void disabledInit() {
    if (m_wasAuto) {
      m_robotContainer.setTeleCurrentLimits();
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Alliance.updateRedAlliance();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    m_wasAuto = true;
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Alliance.updateRedAlliance();

    MatchState.startTeleop();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void simulationPeriodic() {
    // Reset Fuel
    if (SmartDashboard.getBoolean("Reset Fuel Sim", false)) {
      SmartDashboard.putBoolean("Reset Fuel Sim", false);

      m_robotContainer.fuelSim.clearFuel();
      m_robotContainer.fuelSim.spawnStartingFuel();
    }

    m_robotContainer.fuelSim.updateSim();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    if (!m_hasAppliedTestingControls) {
      m_robotContainer.addTestingBindings();
      m_hasAppliedTestingControls = true;
    }

    m_robotContainer.setAllSubsystemTesting(true);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    m_robotContainer.setAllSubsystemTesting(false);
  }

  // Telemetry compatibility shim removed; use Logger.recordOutput directly.
}
