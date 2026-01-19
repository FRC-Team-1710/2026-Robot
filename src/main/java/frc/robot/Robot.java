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
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Alliance;
import frc.robot.constants.MatchState;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot;
import java.util.HashMap;

@Logged
public class Robot extends DynamicTimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final HootAutoReplay hootAutoReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {
    Alliance.updateRedAlliance();

    m_robotContainer = new RobotContainer();
    DataLogManager.start();

    var epilogueConfig = new EpilogueConfiguration();

    epilogueConfig.backend =
        EpilogueBackend.multi(
            new HootEpilogueBackend(), new NTEpilogueBackend(NetworkTableInstance.getDefault()));

    if (isSimulation()) {
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
        Subsystems.Epilogue,
        () ->
            Epilogue.robotLogger.tryUpdate(
                epilogueConfig.backend.getNested(epilogueConfig.root),
                this,
                epilogueConfig.errorHandler),
        epilogueConfig.loggingPeriod,
        epilogueConfig.loggingPeriodOffset);

    addAllSubsystems(m_robotContainer.getAllSubsystems());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    hootAutoReplay.update();

    MatchState.updateAutonomousWinner();
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
  public void simulationPeriodic() {}

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

  /** A map of all subsystems with their period */
  public void addAllSubsystems(HashMap<Subsystems, Pair<Runnable, Time>> subsystems) {
    int id = 0;
    for (Subsystems key : subsystems.keySet()) {
      id++;
      addSubsystem(
          key,
          subsystems.get(key).getFirst(),
          subsystems.get(key).getSecond(),
          Seconds.of(0.02 / Subsystems.values().length).times(id));
    }
  }

  public static EpilogueBackend telemetry() {
    return Epilogue.getConfig().backend;
  }
}
