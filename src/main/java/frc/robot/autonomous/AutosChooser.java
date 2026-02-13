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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedStates;
import java.util.HashMap;

/** Add your docs here. */
public class AutosChooser {
  private static HashMap<Auto, Command> autoCommands;
  private static SendableChooser<Auto> autoChooser;

  // private static Trigger actionsTrigger;
  // private static SendableChooser<Runnable> actions;

  public static FollowPath.Builder pathBuilder;

  private boolean climb = false;
  private boolean depot = false;

  final Trigger Climb = new Trigger(() -> SmartDashboard.getBoolean("Auto/Climb?", climb));
  final Trigger Depot = new Trigger(() -> SmartDashboard.getBoolean("Auto/Depot?", depot));

  public AutosChooser(Superstructure superstructure, CommandSwerveDrivetrain drivetrain) {
    pathBuilder =
        new FollowPath.Builder(
                drivetrain, // The drive subsystem to require
                drivetrain::getPose, // Supplier for current robot pose
                drivetrain::getRobotSpeeds, // Supplier for current speeds
                (speeds) ->
                    drivetrain.setControl(
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

    addPath(Auto.ZONE1, autoPathing(climb, depot, drivetrain).get("Zone 1"));
    addPath(Auto.ZONE3, autoPathing(climb, depot, drivetrain).get("Zone 3"));
    addPath(Auto.RIGHTINSIDE, autoPathing(climb, depot, drivetrain).get("rightinside"));
    addPath(Auto.LEFTINSIDE, autoPathing(climb, depot, drivetrain).get("leftinside"));

    SmartDashboard.putBoolean("Auto/Climb?", climb);
    SmartDashboard.putBoolean("Auto/Depot?", depot);

    // Climb.onChange(Commands.runOnce(() -> addPaths(drivetrain, climb)));

    // Climb.onChange(Commands.runOnce(() -> addPaths(drivetrain, climb)));
    // Depot.onChange(Commands.runOnce(() -> addPaths(drivetrain, depot)));

    // actions.setDefaultOption("neither", this::neither);
    // actions.addOption("Climb", climb = true);
    // actions.addOption("Depot", depot = true);
    // actions.addOption("both", both = true);
    // actions.addOption("neither", both = false);

    // actions.onChange(runnable -> updateActions(runnable));

    SmartDashboard.putData(autoChooser);
    // Put preset autos hare//
    SmartDashboard.putString("Auto/CustomInput", "");
    SmartDashboard.putData("Auto/AutoChooser", autoChooser);
    // SmartDashboard.putData("Auto/Actions", actions);

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

  public static void addPath(Auto auto, Command command) {
    autoCommands.put(auto, command);
    autoChooser.addOption(auto.name(), auto);
  }

  public void addPaths(CommandSwerveDrivetrain drivetrain, Boolean action) {
    if (action == climb) {
      climb = !climb;
    } else if (action == depot) {
      depot = !depot;
    }
    addPath(Auto.ZONE1, autoPathing(climb, depot, drivetrain).get("Zone 1"));
    addPath(Auto.ZONE3, autoPathing(climb, depot, drivetrain).get("Zone 3"));
    addPath(Auto.RIGHTINSIDE, autoPathing(climb, depot, drivetrain).get("rightinside"));
    addPath(Auto.LEFTINSIDE, autoPathing(climb, depot, drivetrain).get("leftinside"));
  }

  public Command getAuto() {

    return autoChooser.getSelected() == Auto.CUSTOM
        ? pathBuilder.build(
            CustomAutoUnpacker.unpack(SmartDashboard.getString("Auto/CustomInput", "")))
        : autoCommands.get(autoChooser.getSelected());
  }

  public Command selectAuto(CommandSwerveDrivetrain drivetrain) {

    // Climb.onChange(Commands.runOnce(() -> addPaths(drivetrain, climb)));
    // Depot.onChange(Commands.runOnce(() -> addPaths(drivetrain, depot)));

    boolean climbValue = SmartDashboard.getBoolean("Auto/Climb?", climb);
    boolean depotValue = SmartDashboard.getBoolean("Auto/Depot?", depot);

    String currentAuto = autoChooser.getSelected().toString();
    SmartDashboard.putString("Auto/Selected", currentAuto);

    return autoPathing(climbValue, depotValue, drivetrain).get(currentAuto);
  }

  public static HashMap<String, Command> autoPathing(
      Boolean climbPath, Boolean depotPath, CommandSwerveDrivetrain drivetrain) {
    HashMap<String, Command> listOfPaths = new HashMap<>();
    listOfPaths.put(
        "ZONE3",
        Commands.sequence(
            pathBuilder.build(new Path("zone3cycleright")),
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
        "rightinside",
        Commands.sequence(
            pathBuilder.build(new Path("rightinside")),
            pathBuilder.build(new Path("zone3climb")).onlyIf(() -> climbPath)));
    listOfPaths.put(
        "leftinside",
        Commands.sequence(
            pathBuilder.build(new Path("leftinside")),
            pathBuilder.build(new Path("zone3climb")).onlyIf(() -> climbPath)));

    return listOfPaths;
  }

  // private void updateActions(Runnable runnable) {
  //   runnable.run();
  // }

  // private void neither() {
  //   // do stuff
  // }

  public enum Auto {
    NONE(),
    CUSTOM(),
    ZONE1(),
    ZONE3(),
    RIGHTINSIDE(),
    LEFTINSIDE(),
  }
}
