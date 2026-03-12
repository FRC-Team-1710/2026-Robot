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
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveStates;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.AddableStates;
import frc.robot.subsystems.Superstructure.WantedStates;
import frc.robot.subsystems.shooter.Shooter;
import java.util.HashMap;

/** Add your docs here. */
public class AutosChooser {
  private static HashMap<Auto, Command> autoCommands;
  private static SendableChooser<Auto> autoChooser;

  public static FollowPath.Builder pathBuilder;

  private static boolean hasResetRotation = false;

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
                new PIDController(1.5, 0.0, 0.0) // Cross-track PID
                )
            .withShouldFlip(() -> Alliance.redAlliance);

    m_depot = false;

    autoCommands = new HashMap<>();
    autoCommands.put(Auto.NONE, Commands.none());

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", Auto.NONE);

    addPath(Auto.ZONE1, autoPathing(m_depot).get("ZONE1"));
    addPath(Auto.ZONE3, autoPathing(m_depot).get("ZONE3"));
    addPath(Auto.RIGHT_INSIDE, autoPathing(m_depot).get("RIGHT_INSIDE"));
    addPath(Auto.LEFT_INSIDE, autoPathing(m_depot).get("LEFT_INSIDE"));
    addPath(Auto.ZONE2, autoPathing(m_depot).get("ZONE2"));

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
          Commands.run(
                  () -> {
                    drivetrain.setAutonomousRequestOverride(true);
                    drivetrain.applyPriorityRequestAuto(
                        drivetrain
                            .fieldCentric
                            .withDriveState(DriveStates.ROTATION_LOCK)
                            .withTargetRotation(superstructure.getRotationForScore())
                            .withVelocityX(0) // ensure previous controls aren't affecting auto
                            .withVelocityY(0)
                            .withRotationalRate(0));
                    superstructure.setAddableState(AddableStates.Jostle);
                    superstructure.setWantedState(WantedStates.ShootAuto);
                    if (!hasResetRotation && superstructure.driveAtTarget()) {
                      drivetrain.setShouldAcceptNextVisionMeasurementRotation(true);
                      hasResetRotation = true;
                    }
                  })
              .schedule();
        });

    FollowPath.registerEventTrigger(
        "EndShoot",
        () -> {
          Commands.waitUntil(() -> shooter.getFPS() < -1)
              .finallyDo(
                  () -> {
                    superstructure.setWantedState(WantedStates.DefaultAuto);
                    drivetrain.setAutonomousRequestOverride(false);
                  })
              .schedule();
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

  public static void addPath(Auto auto, Command command) {
    autoCommands.put(auto, command);
    autoChooser.addOption(auto.name(), auto);
  }

  public Command selectAuto(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
    boolean depotValue = SmartDashboard.getBoolean("Auto/Depot?", m_depot);
    Auto currentAuto = autoChooser.getSelected();

    return autoCommands
        .get(currentAuto)
        .andThen(pathBuilder.build(new Path("depot")).onlyIf(() -> depotValue));
  }

  public static HashMap<String, Command> autoPathing(boolean depotPath) {
    HashMap<String, Command> listOfPaths = new HashMap<>();
    var temp = new Path("outsideracer");
    temp.mirror();
    listOfPaths.put("RIGHT_INSIDE", Commands.sequence(pathBuilder.build(temp)));
    listOfPaths.put("LEFT_INSIDE", Commands.sequence(pathBuilder.build(new Path("outsideracer"))));
    listOfPaths.put(
        "ZONE3",
        Commands.sequence(
            pathBuilder.build(new Path("zone3cycleright")),
            pathBuilder.build(new Path("zone3cycleleft"))));
    listOfPaths.put(
        "ZONE1",
        Commands.sequence(
            pathBuilder.build(new Path("zone1cycleleft"))
            // pathBuilder.build(new Path("zone1cycleright"))
            ));
    listOfPaths.put("ZONE2", Commands.sequence(pathBuilder.build(new Path("zone2"))));

    return listOfPaths;
  }

  public enum Auto {
    NONE(),
    ZONE1(),
    ZONE3(),
    ZONE2(),
    RIGHT_INSIDE(),
    LEFT_INSIDE(),
  }
}
