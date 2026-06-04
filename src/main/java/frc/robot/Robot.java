// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Alliance;
import frc.robot.constants.MatchState;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private boolean m_wasAuto = false;

  private final RobotContainer m_robotContainer;

  private boolean m_hasAppliedTestingControls = false;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Mode.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    Alliance.updateRedAlliance();

    m_robotContainer = new RobotContainer();

    DriverStation.silenceJoystickConnectionWarning(true);

    if (Mode.currentMode == CurrentMode.SIM) {
      SmartDashboard.putBoolean("Sim/Fuel/Reset", false);
    }

    // Lowers brownout threshold to 6.0V
    RobotController.setBrownoutVoltage(6.0);

    DriverStation.silenceJoystickConnectionWarning(true);

    SignalLogger.setPath("/media/sda1");
    SignalLogger.start();

    m_wasAuto = false;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    MatchState.updateAutonomousWinner();
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
  public void teleopExit() {}

  @Override
  public void simulationPeriodic() {
    // Reset Fuel
    if (SmartDashboard.getBoolean("Sim/Fuel/Reset", false)) {
      SmartDashboard.putBoolean("Sim/Fuel/Reset", false);

      m_robotContainer.fuelSim.clearFuel();
      m_robotContainer.fuelSim.spawnStartingFuel();
    }

    m_robotContainer.fuelSim.updateSim();
  }

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
}
