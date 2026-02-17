// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedStates;
import frc.robot.subsystems.shooter.Shooter;
import java.util.HashMap;

/** Add your docs here. */
public class AutosChooser {
  private static HashMap<Auto, Command> autoCommands;
  private static SendableChooser<Auto> autoChooser;

  public static FollowPath.Builder pathBuilder;

  private boolean m_climb;
  private boolean m_depot;

  public AutosChooser(
      Superstructure superstructure, CommandSwerveDrivetrain drivetrain, Shooter shooter) {
    pathBuilder =
        new FollowPath.Builder(
                drivetrain, // The drive subsystem to require
                drivetrain::getPose, // Supplier for current robot pose
                drivetrain::getRobotSpeeds, // Supplier for current speeds
                (speeds) ->
                    drivetrain.applyRequest(
                        drivetrain.bLineRequest.withSpeeds(speeds)), // Consumer to drive the robot
                new PIDController(5.0, 0.0, 0.0), // Translation PID
                new PIDController(3.0, 0.0, 0.0), // Rotation PID
                new PIDController(2.0, 0.0, 0.0) // Cross-track PID
                )
            .withShouldFlip(() -> Alliance.redAlliance);

    m_climb = false;
    m_depot = false;

    autoCommands = new HashMap<>();
    autoCommands.put(Auto.NONE, Commands.none());

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", Auto.NONE);

    addPath(Auto.ZONE1, autoPathing(m_climb, m_depot).get("ZONE1"));
    addPath(Auto.ZONE3, autoPathing(m_climb, m_depot).get("ZONE3"));
    addPath(Auto.RIGHTINSIDE, autoPathing(m_climb, m_depot).get("RIGHTINSIDE"));
    addPath(Auto.LEFTINSIDE, autoPathing(m_climb, m_depot).get("LEFTINSIDE"));

    SmartDashboard.putBoolean("Auto/Climb?", m_climb);
    SmartDashboard.putBoolean("Auto/Depot?", m_depot);
    // Put preset autos hare//
    SmartDashboard.putString("Auto/CustomInput", "");
    SmartDashboard.putData("Auto/AutoChooser", autoChooser);
    // SmartDashboard.putData("Auto/Actions", actions);

    for (WantedStates state : WantedStates.values()) {
      if (state.name().contains("Auto")) {
        FollowPath.registerEventTrigger(state.name(), () -> superstructure.setWantedState(state));
      }
    }

    FollowPath.registerEventTrigger(
        "RemoveOverride",
        () -> {
          drivetrain.setAutonomousRequestOverride(false);
        });

    FollowPath.registerEventTrigger(
        "HoldPosition",
        () -> {
          drivetrain.setAutonomousRequestOverride(true);
          drivetrain.applyPriorityRequestAuto(new SwerveRequest.SwerveDriveBrake());
        });

    FollowPath.registerEventTrigger(
        "Shoot",
        () -> {
          drivetrain.setAutonomousRequestOverride(true);
          drivetrain.applyPriorityRequestAuto(new SwerveRequest.SwerveDriveBrake());
          superstructure
              .setWantedStateCommand(WantedStates.ShootAuto)
              .until(() -> shooter.notShooting());
        });

    FollowPath.registerEventTrigger(
        "ShootWhileMoving",
        () -> {
          superstructure
              .setWantedStateCommand(WantedStates.ShootAuto)
              .until(() -> shooter.notShooting()).;
        });

    FollowPath.registerEventTrigger(
        "Intake",
        () -> {
          drivetrain.setAutonomousRequestOverride(false);
          superstructure
              .setWantedStateCommand(WantedStates.IntakeAuto)
              .until(() -> drivetrain.getRobotSpeeds().vxMetersPerSecond < 0);
          superstructure.setWantedStateCommand(WantedStates.DefaultAuto);
        });

    FollowPath.registerEventTrigger(
        "IntakeDepot",
        () -> {
          drivetrain.setAutonomousRequestOverride(false);
          superstructure
              .setWantedStateCommand(WantedStates.IntakeAuto)
              .until(() -> drivetrain.getRobotSpeeds().vyMetersPerSecond < 0);
          superstructure.setWantedStateCommand(WantedStates.DefaultAuto);
        });

    FollowPath.setPoseLoggingConsumer(
        (data) ->
            Robot.telemetry().log("Auto/" + data.getFirst(), data.getSecond(), Pose2d.struct));
    FollowPath.setTranslationListLoggingConsumer(
        (data) ->
            Robot.telemetry()
                .log("Auto/" + data.getFirst(), data.getSecond(), Translation2d.struct));
    FollowPath.setBooleanLoggingConsumer(
        (data) -> Robot.telemetry().log("Auto/" + data.getFirst(), data.getSecond()));
    FollowPath.setDoubleLoggingConsumer(
        (data) -> Robot.telemetry().log("Auto/" + data.getFirst(), data.getSecond()));
  }

  public void setCustom(Command command) {
    autoCommands.put(Auto.CUSTOM, command);
  }

  public static void addPath(Auto auto, Command command) {
    autoCommands.put(auto, command);
    autoChooser.addOption(auto.name(), auto);
  }

  public Command selectAuto(CommandSwerveDrivetrain drivetrain) {

    boolean m_climbValue = SmartDashboard.getBoolean("Auto/Climb?", m_climb);
    boolean depotValue = SmartDashboard.getBoolean("Auto/Depot?", m_depot);

    String currentAuto = autoChooser.getSelected().toString();
    SmartDashboard.putString("Auto/Selected", currentAuto);

    return autoPathing(m_climbValue, depotValue).get(currentAuto);
  }

  public static HashMap<String, Command> autoPathing(Boolean climbPath, Boolean depotPath) {
    HashMap<String, Command> listOfPaths = new HashMap<>();
    listOfPaths.put(
        "ZONE3",
        Commands.sequence(
            pathBuilder.build(new Path("zone3cycleright")).until(Shooter.notShooting()),
            pathBuilder.build(new Path("zone1cyclestraight")),
            pathBuilder.build(new Path("zone3cycleleft")),
            pathBuilder.build(new Path("zone3climb")).onlyIf(() -> climbPath)));
    listOfPaths.put(
        "ZONE1",
        Commands.sequence(
            pathBuilder.build(new Path("depot")).onlyIf(() -> depotPath),
            pathBuilder.build(new Path("zone1cycleleft")),
            pathBuilder.build(new Path("zone1cyclestraight")),
            pathBuilder.build(new Path("zone1cycleright")),
            pathBuilder.build(new Path("zone1climb")).onlyIf(() -> climbPath)));
    listOfPaths.put(
        "RIGHTINSIDE",
        Commands.sequence(
            pathBuilder.build(new Path("rightinside")),
            pathBuilder.build(new Path("zone3climb")).onlyIf(() -> climbPath)));
    listOfPaths.put(
        "LEFTINSIDE",
        Commands.sequence(
            pathBuilder.build(new Path("leftinside")),
            pathBuilder.build(new Path("zone3climb")).onlyIf(() -> climbPath)));

    return listOfPaths;
  }

  public enum Auto {
    NONE(),
    CUSTOM(),
    ZONE1(),
    ZONE3(),
    RIGHTINSIDE(),
    LEFTINSIDE(),
  }
}
