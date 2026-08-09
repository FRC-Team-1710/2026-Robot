// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Alliance;
import frc.robot.constants.MatchState;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot;

@Logged
public class Robot extends DynamicTimedRobot {
  @Logged(importance = Importance.CRITICAL)
  private final RobotContainer m_robotContainer;

  @NotLogged private boolean m_hasAppliedTestingControls = false;

  public Robot() {
    SignalLogger.setPath("/u/logs");
    SignalLogger.start();

    DataLogManager.start("/u/logs");

    Epilogue.configure(
        config -> {
          config.backend =
              // EpilogueBackend.multi(
              new NTEpilogueBackend(NetworkTableInstance.getDefault());
          // new HootEpilogueBackend());

          if (Mode.currentMode == CurrentMode.SIMULATION) {
            config.minimumImportance = Importance.DEBUG;
            config.errorHandler = ErrorHandler.crashOnError();
          } else {
            config.minimumImportance = Importance.INFO;
            config.errorHandler = ErrorHandler.printErrorMessages();
          }

          config.root = "Robot";
        });

    Alliance.updateRedAlliance();

    m_robotContainer = new RobotContainer();

    registerAllSubsystems(m_robotContainer.getAllSubsystems());

    // Epilogue dislikes the custom DynamicTimedRobot class so we manually run its periodic
    registerSubsystem(
        new SubsystemInfo(
            Subsystems.Epilogue,
            () ->
                Epilogue.robotLogger.tryUpdate(
                    Epilogue.getConfig().backend.getNested(Epilogue.getConfig().root),
                    this,
                    Epilogue.getConfig().errorHandler)));

    if (Mode.currentMode == CurrentMode.SIMULATION) {
      SmartDashboard.putBoolean("Reset Fuel Sim", false);
    }

    // Lowers brownout threshold to 6.0V
    RobotController.setBrownoutVoltage(6.0);

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.autoChooserPeriodic();

    CommandScheduler.getInstance().run();

    if (!MatchState.autonomousWinnerIsRed.isPresent()) {
      MatchState.updateAutonomousWinner();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Alliance.updateRedAlliance();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    MatchState.startTeleop();
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

  @NotLogged
  public static EpilogueBackend telemetry() {
    return Epilogue.getConfig().backend.getNested("Outputs");
  }
}
