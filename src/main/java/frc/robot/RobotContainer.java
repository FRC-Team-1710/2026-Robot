// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutosChooser;
import frc.robot.constants.Alliance;
import frc.robot.constants.MatchState;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.constants.Subsystems;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.AddableStates;
import frc.robot.subsystems.Superstructure.CurrentStates;
import frc.robot.subsystems.Superstructure.WantedStates;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.Feeder.FEEDER_STATE;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOCTRE;
import frc.robot.subsystems.feeder.FeederIOSIM;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexStates;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOCTRE;
import frc.robot.subsystems.indexer.IndexerIOSIM;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeStates;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOCTRE;
import frc.robot.subsystems.intake.IntakeIOSIM;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIOArduino;
import frc.robot.subsystems.leds.LedsIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOCTRE;
import frc.robot.subsystems.shooter.ShooterIOSIM;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOFuel;
import frc.robot.utils.DynamicTimedRobot.SubsystemInfo;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import frc.robot.utils.FuelSim;
import java.util.ArrayList;
import java.util.Arrays;

@Logged
public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_mech = new CommandXboxController(1);

  public FuelSim fuelSim;

  private final AutosChooser m_autoChooser;

  @Logged(importance = Importance.CRITICAL)
  public final CommandSwerveDrivetrain drivetrain;

  /* Create subsystems (uses simulated versions when running in simulation) */
  @Logged(importance = Importance.CRITICAL)
  private final Intake m_intake;

  @Logged(importance = Importance.CRITICAL)
  private final Shooter m_shooter;

  @Logged(importance = Importance.CRITICAL)
  private final Indexer m_indexer;

  @Logged(importance = Importance.CRITICAL)
  private final Feeder m_feeder;

  @Logged(importance = Importance.CRITICAL)
  private final Leds m_leds;

  // Should add logging soon
  @NotLogged private final Vision[] m_cameras;

  @NotLogged private final VisionIOFuel m_fuelCamera;

  @Logged(importance = Importance.CRITICAL)
  private final Superstructure m_superstructure;

  /**
   * Constructs the robot container, initializing all subsystems and configuring bindings.
   *
   * @param consumer the times consumer for dynamic scheduling
   */
  public RobotContainer(TimesConsumer consumer) {
    drivetrain = TunerConstants.createDrivetrain();
    drivetrain.setController(m_driver);

    switch (Mode.currentMode) {
      case REAL:
        m_intake =
            new Intake(new IntakeIOCTRE(), consumer, () -> m_driver.leftBumper().getAsBoolean());
        m_shooter = new Shooter(new ShooterIOCTRE(), consumer);
        m_feeder = new Feeder(new FeederIOCTRE(), consumer);
        m_indexer = new Indexer(new IndexerIOCTRE(), consumer);
        m_leds = new Leds(new LedsIOArduino(), m_shooter);
        m_fuelCamera = new VisionIOFuel("FuelCamera");

        m_cameras =
            // Create a stream of Vision objects from the camera configs
            Arrays.stream(VisionConstants.kPoseCameraConfigs)
                // For each config, create a new Vision subsystem with the appropriate arguments
                .map(
                    config ->
                        new Vision(
                            config.name(),
                            config.robotToCamera(),
                            drivetrain)) // TODO: Fix this stuff :p
                // Collect the stream back into an array of Vision subsystems
                .toArray(Vision[]::new);

        break;

      case SIMULATION:
        m_intake =
            new Intake(new IntakeIOSIM(), consumer, () -> m_driver.leftBumper().getAsBoolean());
        m_shooter = new Shooter(new ShooterIOSIM(), consumer);
        m_feeder = new Feeder(new FeederIOSIM(), consumer);
        m_indexer = new Indexer(new IndexerIOSIM(), consumer);
        m_leds = new Leds(new LedsIOSim(), m_shooter);
        m_cameras = new Vision[0];
        m_fuelCamera = new VisionIOFuel("FuelCamera");
        break;

      default:
        m_intake =
            new Intake(new IntakeIO() {}, consumer, () -> m_driver.leftBumper().getAsBoolean());
        m_shooter = new Shooter(new ShooterIO() {}, consumer);
        m_feeder = new Feeder(new FeederIO() {}, consumer);
        m_indexer = new Indexer(new IndexerIO() {}, consumer);
        m_leds = new Leds(new LedsIO() {}, m_shooter);
        m_cameras = new Vision[0];
        m_fuelCamera = new VisionIOFuel("FuelCamera");
        break;
    }

    drivetrain.setFuelTargetSupplier(m_fuelCamera.yawToLargestTarget);

    m_superstructure =
        new Superstructure(
            m_driver, m_mech, drivetrain, m_intake, m_shooter, m_indexer, m_feeder, m_leds);

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
              m_superstructure.getCurrentState() == CurrentStates.Intake
                  || m_superstructure.getCurrentState() == CurrentStates.IntakeAuto);

      fuelSim.setSubticks(5);

      fuelSim.start();

      fuelSim.enableAirResistance();

      fuelSim.shouldShoot = () -> m_driver.rightTrigger().getAsBoolean();

      m_shooter.setFuelSim(fuelSim);
    }

    m_autoChooser = new AutosChooser(m_superstructure, drivetrain, m_shooter);

    configureBindings();
  }

  /** Adds testing-specific button bindings for subsystem control. */
  public void addTestingBindings() {
    m_driver
        .a()
        .onTrue(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.Intaking)))
        .onFalse(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.Down)));

    m_driver
        .b()
        .onTrue(Commands.runOnce(() -> m_indexer.setStateTesting(IndexStates.Indexing)))
        .onFalse(Commands.runOnce(() -> m_indexer.setStateTesting(IndexStates.Idle)));

    m_driver
        .x()
        .onTrue(Commands.runOnce(() -> m_feeder.setStateTesting(FEEDER_STATE.FEEDING)))
        .onFalse(Commands.runOnce(() -> m_feeder.setStateTesting(FEEDER_STATE.STOP)));

    m_driver
        .y()
        .onTrue(Commands.runOnce(() -> m_shooter.setStateTesting(SHOOTER_STATE.CORNER)))
        .onFalse(Commands.runOnce(() -> m_shooter.setStateTesting(SHOOTER_STATE.IDLE)));

    m_driver
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_shooter.setStateTesting(SHOOTER_STATE.TESTING);
                  m_feeder.setStateTesting(FEEDER_STATE.FEEDING);
                  m_indexer.setStateTesting(IndexStates.Indexing);
                  m_intake.setStateTesting(IntakeStates.Jostle);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  m_shooter.setStateTesting(SHOOTER_STATE.IDLE);
                  m_feeder.setStateTesting(FEEDER_STATE.STOP);
                  m_indexer.setStateTesting(IndexStates.Idle);
                  if (m_intake.getState() == IntakeStates.Jostle) {
                    m_intake.setStateTesting(IntakeStates.Down);
                  }
                }));

    m_driver.povRight().onTrue(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.Up)));
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
    m_driver
        .rightStick()
        .and(m_driver.leftStick())
        .onTrue(
            Commands.runOnce(() -> drivetrain.setShouldAcceptNextVisionMeasurementRotation(true))
                .ignoringDisable(true));

    m_driver
        .rightTrigger()
        .onTrue(
            Commands.runOnce(() -> drivetrain.setShouldAcceptNextVisionMeasurementRotation(true))
                .ignoringDisable(true));

    m_driver
        .start()
        .onTrue(
            Commands.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero))
                .ignoringDisable(true));

    m_driver
        .rightTrigger()
        .and(m_driver.leftTrigger().negate())
        .onTrue(
            m_superstructure
                .setWantedStateCommand(WantedStates.Shoot)
                .alongWith(m_superstructure.setAddableStateCommand(AddableStates.Jostle)));

    m_driver
        .leftTrigger()
        .and(m_driver.rightTrigger().negate())
        .onTrue(m_superstructure.setWantedStateCommand(WantedStates.Intake));

    m_driver
        .leftTrigger()
        .and(m_driver.rightTrigger())
        .onTrue(m_superstructure.setWantedStateCommand(WantedStates.IntakeAndShoot));

    m_driver
        .leftTrigger()
        .negate()
        .and(m_driver.rightTrigger().negate())
        .onTrue(m_superstructure.setWantedStateCommand(WantedStates.Default));

    m_driver
        .x()
        .onTrue(
            m_superstructure
                .setWantedStateCommand(WantedStates.Override)
                .alongWith(Commands.runOnce(() -> m_shooter.override(true, SHOOTER_STATE.TRENCH))));

    m_driver
        .x()
        .onFalse(
            m_superstructure
                .setWantedStateCommand(WantedStates.Default)
                .alongWith(Commands.runOnce(() -> m_shooter.override(false, SHOOTER_STATE.IDLE))));

    m_driver
        .a()
        .onTrue(
            m_superstructure
                .setWantedStateCommand(WantedStates.Override)
                .alongWith(Commands.runOnce(() -> m_shooter.override(true, SHOOTER_STATE.CORNER))));

    m_driver
        .a()
        .onFalse(
            m_superstructure
                .setWantedStateCommand(WantedStates.Default)
                .alongWith(Commands.runOnce(() -> m_shooter.override(false, SHOOTER_STATE.IDLE))));

    m_driver
        .b()
        .onTrue(
            m_superstructure
                .setWantedStateCommand(WantedStates.Override)
                .alongWith(Commands.runOnce(() -> m_shooter.override(true, SHOOTER_STATE.TOWER))));

    m_driver
        .b()
        .onFalse(
            m_superstructure
                .setWantedStateCommand(WantedStates.Default)
                .alongWith(Commands.runOnce(() -> m_shooter.override(false, SHOOTER_STATE.IDLE))));

    m_driver
        .leftTrigger()
        .negate()
        .and(m_driver.rightTrigger().negate())
        .and(m_superstructure::currentStateDoesntUseIntake)
        .onTrue(Commands.runOnce(() -> m_intake.setState(IntakeStates.Down)));

    m_driver
        .povRight()
        .and(m_superstructure::currentStateDoesntUseIntake)
        .onTrue(Commands.runOnce(() -> m_intake.setState(IntakeStates.Up)));

    m_driver
        .povRight()
        .and(m_superstructure::currentStateUsesIntake)
        .onTrue(m_superstructure.setAddableStateCommand(AddableStates.IntakeUp));

    m_driver
        .povRight()
        .and(m_superstructure::currentStateUsesIntake)
        .onFalse(m_superstructure.setAddableStateCommand(AddableStates.Intaking));

    m_driver
        .povLeft()
        .and(m_superstructure::currentStateUsesIntake)
        .onTrue(m_superstructure.setAddableStateCommand(AddableStates.Jostle));

    m_driver
        .povLeft()
        .and(m_superstructure::currentStateUsesIntake)
        .onFalse(m_superstructure.setAddableStateCommand(AddableStates.Intaking));

    m_mech
        .rightBumper()
        .onTrue(
            Commands.runOnce(() -> MatchState.setAutoWinner(Alliance.redAlliance))
                .ignoringDisable(true));

    m_mech
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> MatchState.setAutoWinner(!Alliance.redAlliance))
                .ignoringDisable(true));
  }

  /** Returns the autonomous command to run during autonomous period. */
  @NotLogged
  public Command getAutonomousCommand() {
    return m_autoChooser.selectAuto(drivetrain, m_shooter);
  }

  /** Returns all subsystem info for dynamic scheduling. */
  public SubsystemInfo[] getAllSubsystems() {
    ArrayList<SubsystemInfo> map = new ArrayList<>();
    map.add(
        new SubsystemInfo(
            Subsystems.Vision,
            this::cycleVision,
            Milliseconds.of(20),
            Milliseconds.of((20.0 / Subsystems.values().length))));
    map.add(
        new SubsystemInfo(
            Subsystems.Superstructure,
            m_superstructure::periodic,
            Milliseconds.of(20),
            Milliseconds.of((20.0 / Subsystems.values().length) * 2)));
    map.add(
        new SubsystemInfo(
            Subsystems.Drive,
            drivetrain::periodic,
            Milliseconds.of(20),
            Milliseconds.of((20.0 / Subsystems.values().length) * 3)));
    map.add(
        new SubsystemInfo(
            Subsystems.Shooter,
            m_shooter::periodic,
            Milliseconds.of(60),
            Milliseconds.of((20.0 / Subsystems.values().length) * 4 + (60.0 / 4))));
    map.add(
        new SubsystemInfo(
            Subsystems.Feeder,
            m_feeder::periodic,
            Milliseconds.of(60),
            Milliseconds.of((20.0 / Subsystems.values().length) * 5 + ((60.0 / 4) * 2))));
    map.add(
        new SubsystemInfo(
            Subsystems.Indexer,
            m_indexer::periodic,
            Milliseconds.of(60),
            Milliseconds.of((20.0 / Subsystems.values().length) * 6 + ((60.0 / 4) * 3))));
    map.add(
        new SubsystemInfo(
            Subsystems.Intake,
            m_intake::periodic,
            Milliseconds.of(60),
            Milliseconds.of((20.0 / Subsystems.values().length) * 7 + ((60.0 / 4) * 4))));
    map.add(
        new SubsystemInfo(
            Subsystems.Leds,
            m_leds::periodic,
            Milliseconds.of(20),
            Milliseconds.of((20.0 / Subsystems.values().length) * 8)));
    return map.toArray(new SubsystemInfo[0]);
  }

  private void cycleVision() {
    for (Vision vision : m_cameras) {
      vision.periodic();
    }
  }
}
