// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedStates;
import java.util.HashMap;

/** Add your docs here. */
public class AutosChooser {
  private final HashMap<Auto, Command> autoCommands;
  private final SendableChooser<Auto> autoChooser;

  private final FollowPath.Builder pathBuilder;

  public AutosChooser(Superstructure superstructure, CommandSwerveDrivetrain drivetrain) {
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

    autoCommands = new HashMap<>();
    autoCommands.put(Auto.NONE, Commands.none());

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", Auto.NONE);

    // Add preset autos hare//
    addPath(Auto.CUSTOM, Commands.none());
    addPath(Auto.TEST_PATH, Commands.none());

    SmartDashboard.putString("Auto/CustomInput", "");
    SmartDashboard.putData("Auto/AutoChooser", autoChooser);

    for (WantedStates state : WantedStates.values()) {
      if (state.name().contains("Auto")) {
        FollowPath.registerEventTrigger(state.name(), () -> superstructure.setWantedState(state));
      }
    }

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

  public void addPath(Auto auto, Command command) {
    autoCommands.put(auto, command);
    autoChooser.addOption(auto.name(), auto);
  }

  public Command getAuto() {
    return autoChooser.getSelected() == Auto.CUSTOM
        ? pathBuilder.build(
            CustomAutoUnpacker.unpack(SmartDashboard.getString("Auto/CustomInput", "")))
        : autoCommands.get(autoChooser.getSelected());
  }

  public enum Auto {
    NONE(),
    CUSTOM(),
    TEST_PATH()
  }
}
