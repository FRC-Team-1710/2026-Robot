// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.HootEpilogueBackend;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Alliance;
import frc.robot.constants.MatchState;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot;
import frc.robot.utils.LogEverything;

@Logged
@SuppressWarnings("unused")
public class Robot extends DynamicTimedRobot {
  @Logged(importance = Importance.DEBUG)
  private Command m_autonomousCommand;

  @Logged(importance = Importance.DEBUG)
  private final RobotContainer m_robotContainer;
  @Logged(importance = Importance.DEBUG)
  private final HootAutoReplay hootAutoReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  @Logged(importance = Importance.DEBUG)
  private final PowerDistribution pdhLogging = new PowerDistribution();

  public Robot() {
    Alliance.updateRedAlliance();

    m_robotContainer = new RobotContainer(this::setSubsystemConsumer);
    DataLogManager.start();

    var epilogueConfig = new EpilogueConfiguration();

    epilogueConfig.backend =
        EpilogueBackend.multi(
            new HootEpilogueBackend(), new NTEpilogueBackend(NetworkTableInstance.getDefault()));

    if (Mode.currentMode == CurrentMode.SIMULATION) {
      epilogueConfig.minimumImportance = Importance.DEBUG;
      epilogueConfig.errorHandler = ErrorHandler.crashOnError();
    } else {
      epilogueConfig.minimumImportance = Importance.INFO;
      epilogueConfig.errorHandler = ErrorHandler.printErrorMessages();
    }

    epilogueConfig.root = "Robot";

    epilogueConfig.loggingPeriod = Seconds.of(0.02);
    epilogueConfig.loggingPeriodOffset = Seconds.of(0.02 - (0.02 / Subsystems.values().length));

    Epilogue.configure(
        config -> {
          config = epilogueConfig;
        });

    DriverStation.silenceJoystickConnectionWarning(true);

    // Epilogue dislikes the custom DynamicTimedRobot class so we manually update it
    addSubsystem(
        new SubsystemInfo(
            Subsystems.Epilogue,
            () ->
                Epilogue.robotLogger.tryUpdate(
                    epilogueConfig.backend.getNested(epilogueConfig.root),
                    this,
                    epilogueConfig.errorHandler),
            epilogueConfig.loggingPeriod,
            epilogueConfig.loggingPeriodOffset));

    addAllSubsystems(m_robotContainer.getAllSubsystems());

    SmartDashboard.putBoolean("Reset Fuel Sim", false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    hootAutoReplay.update();

    MatchState.updateAutonomousWinner();

    LogEverything.logEverythingPossible();
  }

  @Override
  public void disabledInit() {}

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
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Alliance.updateRedAlliance();

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
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Logged(importance = Importance.DEBUG)
  public static EpilogueBackend telemetry() {
    return Epilogue.getConfig().backend.getNested("Outputs");
  }
}
