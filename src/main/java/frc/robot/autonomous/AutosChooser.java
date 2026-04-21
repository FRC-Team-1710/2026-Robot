// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.Superstructure.IntakeAddableStates;
import frc.robot.subsystems.Superstructure.ShooterAddableStates;
import frc.robot.subsystems.Superstructure.WantedStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.shooter.Shooter;
import java.util.HashMap;

/** Add your docs here. */
public class AutosChooser {
  private static HashMap<Auto, Command> autoCommands;
  private static SendableChooser<Auto> autoChooser;

  public static FollowPath.Builder pathBuilder;

  private static boolean hasResetRotation = false;

  private boolean m_depot;

  Timer timer = new Timer();

  /**
   * Creates a new AutosChooser that configures autonomous paths and related event triggers.
   *
   * @param superstructure the superstructure subsystem used to manage high-level robot states
   * @param drivetrain the swerve drivetrain subsystem used for autonomous path following
   * @param shooter the shooter subsystem used during autonomous shooting routines
   */
  public AutosChooser(
      Superstructure superstructure,
      CommandSwerveDrivetrain drivetrain,
      Shooter shooter,
      Intake intake) {
    pathBuilder =
        new FollowPath.Builder(
                drivetrain, // The drive subsystem to require
                drivetrain::getPose, // Supplier for current robot pose
                drivetrain::getRobotSpeeds, // Supplier for current speeds
                (speeds) ->
                    drivetrain.applyRequest(
                        drivetrain
                            .fieldCentricBLine
                            .withVelocityX(speeds.vxMetersPerSecond)
                            .withVelocityY(speeds.vyMetersPerSecond)
                            .withRotationalRate(
                                speeds.omegaRadiansPerSecond)), // Consumer to drive the robot
                new PIDController(4.0, 0.0, 0.0), // Translation PID
                new PIDController(6.0, 0.0, 0.25), // Rotation PID
                new PIDController(5.0, 0.0, 0.0) // Cross-track PID
                )
            .withShouldFlip(
                () -> Alliance.redAlliance); // Automatically filps path based on alliance

    m_depot = false;

    autoCommands = new HashMap<>(); // contains a list of all commands that'll happen during auto
    autoCommands.put(Auto.NONE, Commands.none());

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", Auto.NONE);

    var paths = autoPathing(m_depot, superstructure, drivetrain);

    // addPath(Auto.ZONE1, paths.get("ZONE1"));
    // addPath(Auto.ZONE3, paths.get("ZONE3"));
    // addPath(Auto.RIGHT_INSIDE, paths.get("RIGHT_INSIDE"));
    // addPath(Auto.LEFT_INSIDE, paths.get("LEFT_INSIDE"));
    addPath(Auto.RIGHT_INSIDE_PART_2, paths.get("RIGHT_INSIDE_PART_2"));
    addPath(Auto.LEFT_INSIDE_PART_2, paths.get("LEFT_INSIDE_PART_2"));
    addPath(Auto.RIGHT_OUTSIDE_PART_2, paths.get("RIGHT_OUTSIDE_PART_2"));
    addPath(Auto.LEFT_OUTSIDE_PART_2, paths.get("LEFT_OUTSIDE_PART_2"));

    addPath(Auto.OPTIMIZUM, paths.get("OPTIMIZUM"));
    // addPath(Auto.ZONE2, paths.get("ZONE2"));
    // addPath(Auto.MIDDLE, paths.get("MIDDLE"));

    addPath(Auto.TEST, paths.get("TEST"));

    SmartDashboard.putString("Auto/CustomInput", "");
    SmartDashboard.putData("Auto/AutoChooser", autoChooser);

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
        "RaiseIntake",
        () -> {
          intake.setState(IntakeStates.Half);
        });

    FollowPath.registerEventTrigger(
        "HoldPosition",
        () -> {
          drivetrain.setAutonomousRequestOverride(true);
          drivetrain.applyPriorityRequestAuto(new SwerveRequest.SwerveDriveBrake());
        });

    FollowPath.registerEventTrigger(
        "SpinUp", () -> superstructure.setShooterAddableState(ShooterAddableStates.SpinUp));

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

  private Command getShootCommand(
      Superstructure superstructure, CommandSwerveDrivetrain drivetrain) {
    return Commands.runOnce(
            () -> {
              timer.stop();
              timer.reset();
              hasResetRotation = false;
            })
        .andThen(
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
                  superstructure.setWantedState(WantedStates.ShootAuto);
                  if (!hasResetRotation && superstructure.driveAtTarget()) {
                    drivetrain.setShouldAcceptNextVisionMeasurementRotation(true);
                    hasResetRotation = true;
                  }
                  if (superstructure.flywheelAtTarget()) {
                    timer.start(); // Only count actual shooting time
                  }
                  if (timer.get() >= 2.25) {
                    superstructure.setIntakeAddableState(IntakeAddableStates.IntakeUp);
                  } else {
                    superstructure.setIntakeAddableState(IntakeAddableStates.Intaking);
                  }
                }))
        .until(() -> timer.get() > 3.75)
        .finallyDo(
            () -> {
              superstructure.setWantedState(WantedStates.DefaultAuto);
              superstructure.setIntakeAddableState(IntakeAddableStates.Intaking);
              drivetrain.setAutonomousRequestOverride(false);
              superstructure.setShooterAddableState(ShooterAddableStates.Idle);
            });
  }

  public static void addPath(Auto auto, Command command) {
    autoCommands.put(auto, command);
    autoChooser.addOption(auto.name(), auto);
  }

  public Command selectAuto() {
    return autoCommands.get(autoChooser.getSelected());
  }

  public HashMap<String, Command> autoPathing(
      boolean depotPath, Superstructure superstructure, CommandSwerveDrivetrain drivetrain) {
    HashMap<String, Command> listOfPaths = new HashMap<>();
    var temp = new Path("outsideracer");
    var temp2 = new Path("Loopdaloop");
    var temp3 = new Path("insideracer");
    temp.mirror(); // mirrors the path across the y axis\
    temp2.mirror(); // mirrors the path across the y axis\
    temp3.mirror(); // mirrors the path across the y axis\
    // listOfPaths.put(
    //     "RIGHT_INSIDE",
    //     Commands.sequence(pathBuilder.build(temp))); // flipped version of left_inside
    // listOfPaths.put("LEFT_INSIDE", Commands.sequence(pathBuilder.build(new
    // Path("outsideracer"))));
    listOfPaths.put(
        "RIGHT_OUTSIDE_PART_2",
        Commands.sequence(
            pathBuilder.build(temp), pathBuilder.build(temp2))); // flipped version of left_inside
    listOfPaths.put(
        "LEFT_OUTSIDE_PART_2",
        Commands.sequence(
            pathBuilder.build(new Path("outsideracer")),
            pathBuilder.build(new Path("Loopdaloop"))));
    listOfPaths.put(
        "RIGHT_INSIDE_PART_2",
        Commands.sequence(
            pathBuilder.build(temp3), pathBuilder.build(temp2))); // flipped version of left_inside
    listOfPaths.put(
        "LEFT_INSIDE_PART_2",
        Commands.sequence(
            pathBuilder.build(new Path("insideracer")), pathBuilder.build(new Path("Loopdaloop"))));
    listOfPaths.put("TEST", Commands.sequence(pathBuilder.build(new Path("test"))));
    listOfPaths.put(
        "OPTIMIZUM",
        Commands.sequence(
            pathBuilder.build(new Path("optimizum")),
            getShootCommand(superstructure, drivetrain),
            pathBuilder.build(new Path("secondOptimizums")),
            getShootCommand(superstructure, drivetrain)));
    // listOfPaths.put(
    //     "ZONE3",
    //     Commands.sequence(
    //         pathBuilder.build(new Path("zone3cycleright")),
    //         pathBuilder.build(new Path("zone3cycleleft"))));
    // listOfPaths.put("MIDDLE", Commands.sequence(pathBuilder.build(new Path("sweep"))));
    // listOfPaths.put(
    //     "ZONE1",
    //     Commands.sequence(
    //         pathBuilder.build(new Path("zone1cycleleft"))
    //         // pathBuilder.build(new Path("zone1cycleright"))
    //         ));
    // listOfPaths.put("ZONE2", Commands.sequence(pathBuilder.build(new Path("zone2"))));
    // the game
    //  >:(  -Carter

    return listOfPaths;
  }

  // if you make a new path then you need to add the name here
  public enum Auto {
    NONE(),
    ZONE1(),
    ZONE3(),
    ZONE2(),
    LEFT_INSIDE(),
    RIGHT_INSIDE(),
    LEFT_INSIDE_PART_2(),
    RIGHT_INSIDE_PART_2(),
    LEFT_OUTSIDE_PART_2(),
    RIGHT_OUTSIDE_PART_2(),
    MIDDLE(),
    TEST(),
    OPTIMIZUM(),
  }
}
