// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.IntakeAddableStates;
import frc.robot.subsystems.Superstructure.WantedStates;

/** Add your docs here. */
public class AutosChooser {
  private final SendableChooser<Auto> m_autoChooser;

  private final FollowPath.Builder m_pathBuilder;

  private final Timer m_timer = new Timer();

  private Auto m_currentAuto = Auto.LeftCut;
  private Auto m_previousAuto = null;

  private Command m_currentCommand = Commands.none();

  private final Superstructure m_superstructure;
  private final CommandSwerveDrivetrain m_drivetrain;

  private boolean m_autoEnabled = false;

  /**
   * Creates a new AutosChooser that configures autonomous paths and related event triggers.
   *
   * @param superstructure the superstructure subsystem used to manage high-level robot states
   * @param drivetrain the swerve drivetrain subsystem used for autonomous path following
   */
  public AutosChooser(Superstructure superstructure, CommandSwerveDrivetrain drivetrain) {
    m_superstructure = superstructure;
    m_drivetrain = drivetrain;

    m_pathBuilder =
        new FollowPath.Builder(
                drivetrain,
                drivetrain::getPose,
                drivetrain::getRobotSpeeds,
                (speeds) ->
                    drivetrain.applyRequest(
                        drivetrain
                            .m_robotCentricBLine
                            .withVelocityX(speeds.vxMetersPerSecond)
                            .withVelocityY(speeds.vyMetersPerSecond)
                            .withRotationalRate(speeds.omegaRadiansPerSecond)),
                new PIDController(3.0, 0.0, 0.0), // Translation
                new PIDController(4.0, 0.0, 0.0), // Rotation
                new PIDController(4.0, 0.0, 0.0) // Cross-track
                )
            .withShouldFlip(
                () -> Alliance.redAlliance) // Automatically flips path based on alliance
            .withShouldMirror(this::shouldMirror); // Automatically mirrors path based on alliance

    m_autoChooser = new SendableChooser<>();
    for (Auto auto : Auto.values()) {
      m_autoChooser.addOption(auto.name(), auto);
    }

    SmartDashboard.putData("Auto/AutoChooser", m_autoChooser);

    FollowPath.registerEventTrigger(
        "IntakeAuto", () -> superstructure.setWantedState(WantedStates.IntakeAuto));

    m_autoChooser.onChange(this::consumeAutoChooserChange);

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

  private boolean shouldMirror() {
    return m_currentAuto.shouldMirror();
  }

  private void consumeAutoChooserChange(Auto auto) {
    if (auto != null) {
      m_currentAuto = auto;
    }
  }

  private void buildAuto() {
    switch (m_currentAuto) {
      case Test:
        m_currentCommand = m_pathBuilder.build(m_currentAuto.startingPath).ignoringDisable(true);
        return;
      case igbro:
        m_currentCommand =
            m_pathBuilder
                .build(m_currentAuto.startingPath)
                .andThen(Commands.waitSeconds(5))
                .andThen(getShootCommand(m_superstructure, m_drivetrain))
                .finallyDo(
                    () -> {
                      m_superstructure.setWantedState(WantedStates.Default);
                    })
                .ignoringDisable(true);
        return;
      default:
        m_currentCommand =
            m_pathBuilder
                .build(m_currentAuto.startingPath)
                .andThen(getShootCommand(m_superstructure, m_drivetrain))
                .andThen(m_pathBuilder.build(new Path("main2nd")))
                .andThen(getShootCommand(m_superstructure, m_drivetrain))
                .andThen(m_pathBuilder.build(new Path("main3rd")))
                .finallyDo(
                    () -> {
                      m_superstructure.setWantedState(WantedStates.Default);
                    })
                .ignoringDisable(true);
        return;
    }
  }

  @SuppressWarnings("removal")
  public void periodic() {
    if (m_autoEnabled && !m_currentCommand.isScheduled()) {
      return;
    }
    if (m_currentCommand.isScheduled()
        && DriverStation.isEnabled()
        && !DriverStation.isAutonomous()) {
      m_currentCommand.cancel();
      m_autoEnabled = true;
      return;
    }
    if (m_currentAuto != m_previousAuto) {
      m_previousAuto = m_currentAuto;
      if (m_currentCommand.isScheduled()) {
        m_currentCommand.cancel();
      }
      buildAuto();
      m_drivetrain.resetPose(
          (m_currentAuto.shouldMirror != Alliance.redAlliance)
              ? new Pose2d(
                  new Translation2d(
                      Alliance.redAlliance
                          ? FieldConstants.kFieldLength.in(Meters)
                              - m_currentAuto.startingPath.getStartPose().getX()
                          : m_currentAuto.startingPath.getStartPose().getX(),
                      FieldConstants.kFieldWidth.in(Meters)
                          - m_currentAuto.startingPath.getStartPose().getY()),
                  m_currentAuto.startingPath.getStartPose().getRotation().unaryMinus())
              : new Pose2d(
                  new Translation2d(
                      Alliance.redAlliance
                          ? FieldConstants.kFieldLength.in(Meters)
                              - m_currentAuto.startingPath.getStartPose().getX()
                          : m_currentAuto.startingPath.getStartPose().getX(),
                      m_currentAuto.startingPath.getStartPose().getY()),
                  m_currentAuto.startingPath.getStartPose().getRotation()));
      m_currentCommand.schedule();
    }

    if (DriverStation.isAutonomousEnabled()) {
      m_autoEnabled = true;
    }

    if (DriverStation.isDisabled() && m_autoEnabled && m_currentCommand.isScheduled()) {
      m_currentCommand.cancel();
    }

    Robot.telemetry().log("Auto/CurrentAuto", m_currentAuto.name());
    Robot.telemetry().log("Auto/CurrentCommandIsScheduled", m_currentCommand.isScheduled());
  }

  private Command getShootCommand(
      Superstructure superstructure, CommandSwerveDrivetrain drivetrain) {
    return Commands.runOnce(
            () -> {
              m_timer.stop();
              m_timer.reset();
              drivetrain.setAutonomousRequestOverride(true);
              superstructure.setWantedState(WantedStates.ScoreAuto);
            })
        .andThen(
            Commands.run(
                () -> {
                  drivetrain.applyPriorityRequestAuto(
                      drivetrain
                          .m_fieldCentricFacingAngle
                          .withTargetDirection(superstructure.getRotationForScore())
                          .withVelocityX(0)
                          .withVelocityY(0));
                  if (superstructure.readyToShoot()) {
                    m_timer.start(); // Flywheel should be at target but drivetrain might not be
                  }
                  if (m_timer.get() >= 0.75) {
                    superstructure.setIntakeAddableState(IntakeAddableStates.IntakeUp);
                  } else {
                    superstructure.setIntakeAddableState(IntakeAddableStates.Intaking);
                  }
                }))
        .until(() -> m_timer.get() > 2.5)
        .finallyDo(
            () -> {
              drivetrain.setAutonomousRequestOverride(false);
              superstructure.setWantedState(WantedStates.DefaultAuto);
              superstructure.setIntakeAddableState(IntakeAddableStates.Intaking);
            });
  }

  public enum Auto {
    Test(new Path("tuningpath")),
    LeftFar(new Path("supaYummies")),
    LeftNear(new Path("miniYummies")),
    LeftCut(new Path("myYummies")),
    LeftFarNoTap(new Path("supaYummiesNoWall")),
    LeftNearNoTap(new Path("miniYummiesNoWall")),
    LeftCutNoTap(new Path("myYummiesNoWall")),
    RightFar(true, new Path("supaYummies")),
    RightNear(true, new Path("miniYummies")),
    RightCut(true, new Path("myYummies")),
    RightFarNoTap(true, new Path("supaYummiesNoWall")),
    RightNearNoTap(true, new Path("miniYummiesNoWall")),
    igbro(new Path("iguessbro")),
    RightCutNoTap(true, new Path("myYummiesNoWall"));

    public final boolean shouldMirror;
    public final Path startingPath;

    Auto(Path startingPath) {
      this(false, startingPath);
    }

    Auto(boolean shouldMirror, Path startingPath) {
      this.shouldMirror = shouldMirror;
      this.startingPath = startingPath;
    }

    public boolean shouldMirror() {
      return shouldMirror;
    }
  }
}
