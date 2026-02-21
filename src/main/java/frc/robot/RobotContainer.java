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
import frc.robot.subsystems.Superstructure.CurrentStates;
import frc.robot.subsystems.Superstructure.WantedStates;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOCTRE;
import frc.robot.subsystems.feeder.FeederIOSIM;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOCTRE;
import frc.robot.subsystems.indexer.IndexerIOSIM;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOCTRE;
import frc.robot.subsystems.intake.IntakeIOSIM;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIOCTRE;
import frc.robot.subsystems.leds.LedsIOSIM;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOCTRE;
import frc.robot.subsystems.shooter.ShooterIOSIM;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.DynamicTimedRobot.SubsystemInfo;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import frc.robot.utils.FuelSim;
import java.util.ArrayList;
import java.util.Arrays;

@Logged
public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController mech = new CommandXboxController(1);

  public FuelSim fuelSim;

  private final AutosChooser autoChooser;

  @Logged(importance = Importance.CRITICAL)
  public final CommandSwerveDrivetrain drivetrain;

  /* Create subsystems (uses simulated versions when running in simulation) */
  @Logged(importance = Importance.CRITICAL)
  private final Intake intake;

  @Logged(importance = Importance.CRITICAL)
  private final Shooter shooter;

  @Logged(importance = Importance.CRITICAL)
  private final Indexer indexer;

  @Logged(importance = Importance.CRITICAL)
  private final Feeder feeder;

  @Logged(importance = Importance.CRITICAL)
  private final Leds leds;

  // Should add logging soon
  @NotLogged private Vision[] cameras;

  @Logged(importance = Importance.CRITICAL)
  private final Superstructure superstructure;

  public RobotContainer(TimesConsumer consumer) {
    drivetrain = TunerConstants.createDrivetrain();
    drivetrain.setController(driver);

    switch (Mode.currentMode) {
      case REAL:
        intake = new Intake(new IntakeIOCTRE(), consumer);
        shooter = new Shooter(new ShooterIOCTRE(), consumer);
        feeder = new Feeder(new FeederIOCTRE(), consumer);
        indexer =
            new Indexer(new IndexerIOCTRE(), consumer, () -> driver.leftBumper().getAsBoolean());
        leds = new Leds(new LedsIOCTRE(), shooter);

        cameras =
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
        intake = new Intake(new IntakeIOSIM(), consumer);
        shooter = new Shooter(new ShooterIOSIM(), consumer);
        feeder = new Feeder(new FeederIOSIM(), consumer);
        indexer =
            new Indexer(new IndexerIOSIM(), consumer, () -> driver.leftBumper().getAsBoolean());
        leds = new Leds(new LedsIOSIM(), shooter);
        break;

      default:
        intake = new Intake(new IntakeIO() {}, consumer);
        shooter = new Shooter(new ShooterIO() {}, consumer);
        feeder = new Feeder(new FeederIO() {}, consumer);
        indexer =
            new Indexer(new IndexerIO() {}, consumer, () -> driver.leftBumper().getAsBoolean());
        leds = new Leds(new LedsIO() {}, shooter);
        break;
    }

    superstructure =
        new Superstructure(driver, mech, drivetrain, intake, shooter, indexer, feeder, leds);

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
          () -> superstructure.getCurrentState() == CurrentStates.Intake);

      fuelSim.setSubticks(5);

      fuelSim.start();

      fuelSim.enableAirResistance();
    }

    autoChooser = new AutosChooser(superstructure, drivetrain);

    superstructure.setFuelSim(fuelSim);

    configureBindings();
  }

  private void configureBindings() {
    driver
        .start()
        .onTrue(
            Commands.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero))
                .ignoringDisable(true));

    driver
        .rightTrigger()
        .and(driver.leftTrigger().negate())
        .onTrue(superstructure.setWantedStateCommand(WantedStates.Shoot));

    driver
        .leftTrigger()
        .and(driver.rightTrigger().negate())
        .onTrue(superstructure.setWantedStateCommand(WantedStates.Intake));

    driver
        .leftTrigger()
        .and(driver.rightTrigger())
        .onTrue(superstructure.setWantedStateCommand(WantedStates.IntakeAndShoot));

    driver
        .leftTrigger()
        .negate()
        .and(driver.rightTrigger().negate())
        .onTrue(superstructure.setWantedStateCommand(WantedStates.Default));

    mech.rightBumper()
        .onTrue(
            Commands.runOnce(() -> MatchState.setAutoWinner(Alliance.redAlliance))
                .ignoringDisable(true));

    mech.leftBumper()
        .onTrue(
            Commands.runOnce(() -> MatchState.setAutoWinner(!Alliance.redAlliance))
                .ignoringDisable(true));
  }

  @NotLogged
  public Command getAutonomousCommand() {
    return autoChooser.selectAuto(drivetrain);
  }

  public SubsystemInfo[] getAllSubsystems() {
    ArrayList<SubsystemInfo> map = new ArrayList<>();
    map.add(
        new SubsystemInfo(
            Subsystems.Superstructure,
            superstructure::periodic,
            Milliseconds.of(20),
            Milliseconds.of((20.0 / Subsystems.values().length) * 1)));
    map.add(
        new SubsystemInfo(
            Subsystems.Drive,
            drivetrain::periodic,
            Milliseconds.of(20),
            Milliseconds.of((20.0 / Subsystems.values().length) * 2)));
    map.add(
        new SubsystemInfo(
            Subsystems.Intake,
            intake::periodic,
            Milliseconds.of(60),
            Milliseconds.of((20.0 / Subsystems.values().length) * 3 + 20.0)));
    map.add(
        new SubsystemInfo(
            Subsystems.Shooter,
            shooter::periodic,
            Milliseconds.of(60),
            Milliseconds.of((20.0 / Subsystems.values().length) * 4 + 40.0)));
    map.add(
        new SubsystemInfo(
            Subsystems.Indexer,
            indexer::periodic,
            Milliseconds.of(60),
            Milliseconds.of((20.0 / Subsystems.values().length) * 5 + 60.0)));
    map.add(
        new SubsystemInfo(
            Subsystems.Feeder,
            feeder::periodic,
            Milliseconds.of(60),
            Milliseconds.of((20.0 / Subsystems.values().length) * 5 + 60.0)));
    map.add(
        new SubsystemInfo(
            Subsystems.Leds,
            leds::periodic,
            Milliseconds.of(20),
            Milliseconds.of((20.0 / Subsystems.values().length) * 7)));
    return map.toArray(new SubsystemInfo[0]);
  }

  private void cycleVision() {
    for (Vision vision : cameras) {
      vision.periodic();
    }
  }
}
