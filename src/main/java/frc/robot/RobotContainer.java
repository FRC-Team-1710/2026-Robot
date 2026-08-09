// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutosChooser;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.constants.SubsystemConstants.VisionConstants;
import frc.robot.constants.Subsystems;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentStates;
import frc.robot.subsystems.Superstructure.IntakeAddableStates;
import frc.robot.subsystems.Superstructure.WantedStates;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FeederStates;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOCTRE;
import frc.robot.subsystems.feeder.FeederIOSIM;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOCTRE;
import frc.robot.subsystems.indexer.IndexerIOSIM;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOCTRE;
import frc.robot.subsystems.intake.IntakeIOSIM;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterStates;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOCTRE;
import frc.robot.subsystems.shooter.ShooterIOSIM;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.DynamicTimedRobot.SubsystemInfo;
import frc.robot.utils.FuelSim;
import java.util.ArrayList;
import java.util.Arrays;

@Logged
public class RobotContainer {
  @NotLogged private final CommandXboxController m_driver = new CommandXboxController(0);
  @NotLogged private final CommandXboxController m_mech = new CommandXboxController(1);

  @NotLogged public FuelSim fuelSim;

  @NotLogged private final AutosChooser m_autoChooser;

  @Logged(importance = Importance.CRITICAL)
  public final CommandSwerveDrivetrain drivetrain;

  /* Create subsystems (uses simulated versions when running in simulation) */
  @Logged(importance = Importance.CRITICAL)
  private final Intake m_intake;

  @Logged(importance = Importance.CRITICAL)
  private final Shooter m_shooter;

  @NotLogged // Nothing to log, all captured in the .hoot logs
  private final Indexer m_indexer;

  @NotLogged // Nothing to log, all captured in the .hoot logs
  private final Feeder m_feeder;

  @NotLogged private final Leds m_leds; // Everything is logged through Robot.telemetry().log()

  // Epilogue can't log arrays of classes (except for some primitives), so we log each camera
  // separately
  @NotLogged private final Vision[] m_cameras;

  @Logged(importance = Importance.CRITICAL)
  private final Superstructure m_superstructure;

  /** Constructs the robot container, initializing all subsystems and configuring bindings. */
  public RobotContainer() {
    drivetrain = TunerConstants.createDrivetrain();
    drivetrain.setController(m_driver);

    switch (Mode.currentMode) {
      case REAL:
        m_intake = new Intake(new IntakeIOCTRE(), () -> m_driver.leftBumper().getAsBoolean());
        m_shooter = new Shooter(new ShooterIOCTRE());
        m_feeder = new Feeder(new FeederIOCTRE());
        m_indexer = new Indexer(new IndexerIOCTRE());

        m_cameras =
            // Create a stream of Vision objects from the camera configs
            Arrays.stream(VisionConstants.Hardware.kPoseCameraConfigs)
                // For each config, create a new Vision subsystem with the appropriate arguments
                .map(config -> new Vision(config.name(), config.robotToCamera(), drivetrain))
                // Collect the stream back into an array of Vision subsystems
                .toArray(Vision[]::new);
        break;

      case SIMULATION:
        m_intake = new Intake(new IntakeIOSIM(), () -> m_driver.leftBumper().getAsBoolean());
        m_shooter = new Shooter(new ShooterIOSIM());
        m_feeder = new Feeder(new FeederIOSIM());
        m_indexer = new Indexer(new IndexerIOSIM());
        // m_cameras = new Vision[0];
        m_cameras =
            // Create a stream of Vision objects from the camera configs
            Arrays.stream(VisionConstants.Hardware.kPoseCameraConfigs)
                // For each config, create a new Vision subsystem with the appropriate arguments
                .map(config -> new Vision(config.name(), config.robotToCamera(), drivetrain))
                // Collect the stream back into an array of Vision subsystems
                .toArray(Vision[]::new);
        break;

      default:
        m_intake = new Intake(new IntakeIO() {}, () -> m_driver.leftBumper().getAsBoolean());
        m_shooter = new Shooter(new ShooterIO() {});
        m_feeder = new Feeder(new FeederIO() {});
        m_indexer = new Indexer(new IndexerIO() {});
        m_cameras = new Vision[0];
        break;
    }

    m_superstructure =
        new Superstructure(m_driver, drivetrain, m_intake, m_shooter, m_indexer, m_feeder);

    m_leds = new Leds(m_superstructure);

    // Fuel Simulation
    if (Mode.currentMode == CurrentMode.SIMULATION) {
      fuelSim = new FuelSim("FuelSim");
      fuelSim.spawnStartingFuel();

      double width = Units.inchesToMeters(39.875);
      double length = Units.inchesToMeters(27.875);

      fuelSim.registerRobot(
          width,
          length,
          Units.inchesToMeters(6.75),
          drivetrain::getPose,
          drivetrain::getFieldSpeeds);

      fuelSim.registerIntake(
          width / 2,
          width / 2 + Units.inchesToMeters(10), // Intake is 10 inches from the edge
          -length / 2,
          length / 2,
          () ->
              m_driver.leftTrigger().getAsBoolean()
                  || m_superstructure.getCurrentState() == CurrentStates.IntakeAuto);

      fuelSim.setSubticks(5);

      fuelSim.start();

      fuelSim.enableAirResistance();

      fuelSim.shouldScore =
          () ->
              switch (m_superstructure.getCurrentState()) {
                case ScoreWhileIntaking,
                        ScoreWhileIntakingAuto,
                        ScoreWithIntakeUp,
                        ScoreWithIntakeUpAuto ->
                    true;
                default -> false;
              };

      fuelSim.shouldShoot =
          () ->
              switch (m_superstructure.getCurrentState()) {
                case PassWhileIntaking,
                        PassWithIntakeUp,
                        ScoreWhileIntaking,
                        ScoreWhileIntakingAuto,
                        ScoreWithIntakeUp,
                        ScoreWithIntakeUpAuto ->
                    true;
                default -> false;
              };

      m_shooter.setFuelSim(fuelSim);
    }

    m_autoChooser = new AutosChooser(m_superstructure, drivetrain);

    configureBindings();
  }

  public void autoChooserPeriodic() {
    m_autoChooser.periodic();
  }

  /** Adds testing-specific button bindings for subsystem control. */
  public void addTestingBindings() {
    m_mech
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.Intake)))
        .onFalse(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.Down)));

    m_mech
        .a()
        .onTrue(Commands.runOnce(() -> m_indexer.setStateTesting(IndexerStates.Run)))
        .onFalse(Commands.runOnce(() -> m_indexer.setStateTesting(IndexerStates.Idle)));

    m_mech
        .x()
        .onTrue(Commands.runOnce(() -> m_feeder.setStateTesting(FeederStates.Run)))
        .onFalse(Commands.runOnce(() -> m_feeder.setStateTesting(FeederStates.Idle)));

    m_mech
        .y()
        .onTrue(Commands.runOnce(() -> m_shooter.setStateTesting(ShooterStates.Test)))
        .onFalse(Commands.runOnce(() -> m_shooter.setStateTesting(ShooterStates.Stop)));

    m_mech
        .b()
        .onTrue(Commands.runOnce(() -> m_shooter.setStateTesting(ShooterStates.TestFast)))
        .onFalse(Commands.runOnce(() -> m_shooter.setStateTesting(ShooterStates.Stop)));

    m_mech
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_shooter.setStateTesting(ShooterStates.Test);
                  m_feeder.setStateTesting(FeederStates.Run);
                  m_indexer.setStateTesting(IndexerStates.Run);
                  m_intake.setStateTesting(IntakeStates.Intake);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_shooter.setStateTesting(ShooterStates.Stop);
                  m_feeder.setStateTesting(FeederStates.Idle);
                  m_indexer.setStateTesting(IndexerStates.Idle);
                  m_intake.setStateTesting(IntakeStates.Down);
                }));

    m_mech
        .povRight()
        .onTrue(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.UpAndIntake)));
  }

  /**
   * Enables or disables testing mode for all subsystems.
   *
   * @param testing true to enable testing mode
   */
  public void setAllSubsystemTesting(boolean testing) {
    m_shooter.setTesting(testing);
    m_intake.setTesting(testing);
    m_indexer.setTesting(testing);
    m_feeder.setTesting(testing);
  }

  private void configureBindings() {
    // General bindings

    m_driver
        .rightStick()
        .and(m_driver.leftStick())
        .onTrue(
            Commands.runOnce(() -> drivetrain.setShouldAcceptNextVisionMeasurementRotation(true))
                .ignoringDisable(true));

    m_driver
        .start()
        .onTrue(
            Commands.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero))
                .ignoringDisable(true));

    // Main bindings

    final Timer m_shootingTimer = new Timer();
    m_driver
        .rightTrigger()
        .onTrue(m_superstructure.setWantedStateCommand(WantedStates.Shoot))
        .and(m_driver.leftTrigger().negate())
        .whileTrue(
            Commands.run(
                    () -> {
                      if (m_superstructure.readyToShoot()) {
                        m_shootingTimer.start();
                      }
                      if (m_shootingTimer.hasElapsed(1.0)) {
                        m_superstructure.setIntakeAddableState(IntakeAddableStates.IntakeUp);
                      }
                    })
                .finallyDo(
                    () -> {
                      m_superstructure.setIntakeAddableState(IntakeAddableStates.Intaking);
                      m_shootingTimer.stop();
                      m_shootingTimer.reset();
                    }));

    m_driver
        .leftTrigger()
        .and(m_driver.rightTrigger().negate()) // Shoot overrides intake
        .onTrue(m_superstructure.setWantedStateCommand(WantedStates.Intake));

    m_driver
        .leftTrigger()
        .and(m_driver.rightTrigger()) // Shoot overrides intake
        .onTrue(m_superstructure.setIntakeAddableStateCommand(IntakeAddableStates.Intaking));

    m_driver
        .leftTrigger()
        .negate()
        .and(m_driver.rightTrigger().negate())
        .onTrue(m_superstructure.setWantedStateCommand(WantedStates.Default));

    m_driver
        .povRight()
        .onTrue(m_superstructure.setIntakeAddableStateCommand(IntakeAddableStates.IntakeUp))
        .onFalse(m_superstructure.setIntakeAddableStateCommand(IntakeAddableStates.Intaking));

    m_driver
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivetrain.toggleBrownout();
                  m_feeder.toggleBrownout();
                  m_indexer.toggleBrownout();
                  m_intake.toggleBrownout();
                }));

    // Backup bindings

    m_driver
        .y()
        .onTrue(
            m_superstructure
                .setWantedStateCommand(WantedStates.Override)
                .alongWith(Commands.runOnce(() -> m_shooter.override(true, ShooterStates.Trench))))
        .onFalse(
            m_superstructure
                .setWantedStateCommand(WantedStates.Default)
                .alongWith(
                    Commands.runOnce(() -> m_shooter.override(false, ShooterStates.IdleScore))));

    m_driver
        .a()
        .onTrue(
            m_superstructure
                .setWantedStateCommand(WantedStates.Override)
                .alongWith(Commands.runOnce(() -> m_shooter.override(true, ShooterStates.Tower))))
        .onFalse(
            m_superstructure
                .setWantedStateCommand(WantedStates.Default)
                .alongWith(
                    Commands.runOnce(() -> m_shooter.override(false, ShooterStates.IdleScore))));

    m_driver
        .x()
        .onTrue(
            m_superstructure
                .setWantedStateCommand(WantedStates.Override)
                .alongWith(
                    Commands.runOnce(() -> m_shooter.override(true, ShooterStates.TowerLeft))))
        .onFalse(
            m_superstructure
                .setWantedStateCommand(WantedStates.Default)
                .alongWith(
                    Commands.runOnce(() -> m_shooter.override(false, ShooterStates.IdleScore))));

    m_driver
        .b()
        .onTrue(
            m_superstructure
                .setWantedStateCommand(WantedStates.Override)
                .alongWith(
                    Commands.runOnce(() -> m_shooter.override(true, ShooterStates.TowerRight))))
        .onFalse(
            m_superstructure
                .setWantedStateCommand(WantedStates.Default)
                .alongWith(
                    Commands.runOnce(() -> m_shooter.override(false, ShooterStates.IdleScore))));

    // SysId bindings

    // m_driver
    //     .a()
    //     .onTrue(Commands.runOnce(() -> drivetrain.sysid(true)))
    //     .whileTrue(drivetrain.sysIdDynamic(Direction.kForward))
    //     .onFalse(Commands.runOnce(() -> drivetrain.sysid(false)));
    // m_driver
    //     .b()
    //     .onTrue(Commands.runOnce(() -> drivetrain.sysid(true)))
    //     .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse))
    //     .onFalse(Commands.runOnce(() -> drivetrain.sysid(false)));
    // m_driver
    //     .x()
    //     .onTrue(Commands.runOnce(() -> drivetrain.sysid(true)))
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward))
    //     .onFalse(Commands.runOnce(() -> drivetrain.sysid(false)));
    // m_driver
    //     .y()
    //     .onTrue(Commands.runOnce(() -> drivetrain.sysid(true)))
    //     .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse))
    //     .onFalse(Commands.runOnce(() -> drivetrain.sysid(false)));
  }

  /** Returns all subsystem info for dynamic scheduling. */
  public SubsystemInfo[] getAllSubsystems() {
    ArrayList<SubsystemInfo> map = new ArrayList<>();
    map.add(new SubsystemInfo(Subsystems.Vision, this::cycleVision));
    map.add(new SubsystemInfo(Subsystems.Drive, drivetrain::periodic));
    map.add(new SubsystemInfo(Subsystems.Superstructure, m_superstructure::periodic));
    // map.add(new SubsystemInfo(Subsystems.Leds, m_leds::periodic));
    return map.toArray(new SubsystemInfo[0]);
  }

  private void cycleVision() {
    for (Vision camera : m_cameras) {
      camera.periodic();
    }
  }
}
