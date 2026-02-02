// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoPathBuilder;
import frc.robot.autonomous.AutosChooser;
import frc.robot.constants.Mode;
import frc.robot.constants.Subsystems;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedStates;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOCTRE;
import frc.robot.subsystems.feeder.FeederIOSIM;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOCTRE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOCTRE;
import frc.robot.subsystems.intake.IntakeIOSIM;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOCTRE;
import frc.robot.subsystems.shooter.ShooterIOSIM;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;
import java.util.HashMap;

@Logged
public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController mech = new CommandXboxController(1);

  private final AutosChooser autoChooser;

  public final CommandSwerveDrivetrain drivetrain;

  /* Create subsystems (uses simulated versions when running in simulation) */
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Feeder feeder;

  private final Superstructure superstructure;

  public RobotContainer(TimesConsumer consumer) {
    drivetrain = TunerConstants.createDrivetrain();
    drivetrain.setController(driver);
    AutoPathBuilder.setDrivetrainInstance(drivetrain);

    switch (Mode.currentMode) {
      case REAL:
        intake = new Intake(new IntakeIOCTRE(), consumer);
        shooter = new Shooter(new ShooterIOCTRE(), consumer);
        indexer = new Indexer(new IndexerIOCTRE(), consumer);
        feeder = new Feeder(new FeederIOCTRE(), consumer);
        break;

      case SIMULATION:
        intake = new Intake(new IntakeIOSIM(), consumer);
        shooter = new Shooter(new ShooterIOSIM(), consumer);
        feeder = new Feeder(new FeederIOSIM(), consumer);
        // TODO: Add IndexerIOSIM
        indexer = new Indexer(new IndexerIO() {}, consumer);
        break;

      default:
        intake = new Intake(new IntakeIO() {}, consumer);
        shooter = new Shooter(new ShooterIO() {}, consumer);
        feeder = new Feeder(new FeederIO(), consumer);
        indexer = new Indexer(new IndexerIO() {}, consumer);
        break;
    }

    superstructure = new Superstructure(driver, mech, drivetrain, intake, shooter, indexer);

    autoChooser = new AutosChooser(superstructure);

    configureBindings();
  }

  private void configureBindings() {
    driver
        .start()
        .onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(Feet.of(0), Feet.of(0), Rotation2d.kZero))));

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
  }

  public Command getAutonomousCommand() {
    return autoChooser.getAuto();
  }

  public HashMap<Subsystems, Pair<Runnable, Pair<Time, Time>>> getAllSubsystems() {
    HashMap<Subsystems, Pair<Runnable, Pair<Time, Time>>> map = new HashMap<>();
    map.put(
        Subsystems.Superstructure,
        new Pair<Runnable, Pair<Time, Time>>(
            superstructure::periodic,
            new Pair<Time, Time>(
                Milliseconds.of(20), Milliseconds.of((20.0 / Subsystems.values().length) * 1))));
    map.put(
        Subsystems.Intake,
        new Pair<Runnable, Pair<Time, Time>>(
            intake::periodic,
            new Pair<Time, Time>(
                Milliseconds.of(60),
                Milliseconds.of((20.0 / Subsystems.values().length) * 2 + 20.0))));
    map.put(
        Subsystems.Shooter,
        new Pair<Runnable, Pair<Time, Time>>(
            shooter::periodic,
            new Pair<Time, Time>(
                Milliseconds.of(60),
                Milliseconds.of((20.0 / Subsystems.values().length) * 3 + 40.0))));
    map.put(
        Subsystems.Indexer,
        new Pair<Runnable, Pair<Time, Time>>(
            indexer::periodic,
            new Pair<Time, Time>(
                Milliseconds.of(60),
                Milliseconds.of((20.0 / Subsystems.values().length) * 4 + 60.0))));
    map.put(
        Subsystems.Drive,
        new Pair<Runnable, Pair<Time, Time>>(
            drivetrain::periodic,
            new Pair<Time, Time>(
                Milliseconds.of(20), Milliseconds.of((20.0 / Subsystems.values().length) * 5))));
    return map;
  }
}
