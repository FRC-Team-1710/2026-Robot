// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutosChooser;
import frc.robot.constants.DrivetrainAccelerationLimits;
import frc.robot.constants.DrivetrainAutomationConstants;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentStates;
import frc.robot.subsystems.Superstructure.IntakeAddableStates;
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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_STATE;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOCTRE;
import frc.robot.subsystems.shooter.ShooterIOSIM;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.FuelSim;
import java.util.Arrays;

public class RobotContainer {
  private final CommandXboxController m_driver = new CommandXboxController(0);
  private final CommandXboxController m_testing = new CommandXboxController(1);

  public FuelSim fuelSim;

  private final AutosChooser m_autoChooser;

  private boolean m_hasntAcceptedVisionRotation = true;

  public final CommandSwerveDrivetrain drivetrain;
  private final Intake m_intake;
  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private final Feeder m_feeder;
  private final Leds m_leds;
  private final Vision[] m_cameras; // Should add logging soon

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
            Arrays.stream(VisionConstants.kPoseCameraConfigs)
                .map(config -> new Vision(config.name(), config.robotToCamera(), drivetrain))
                .toArray(Vision[]::new);

        break;

      case SIM:
        m_intake = new Intake(new IntakeIOSIM(), () -> m_driver.leftBumper().getAsBoolean());
        m_shooter = new Shooter(new ShooterIOSIM());
        m_feeder = new Feeder(new FeederIOSIM());
        m_indexer = new Indexer(new IndexerIOSIM());
        m_cameras = new Vision[0];
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
        new Superstructure(
            m_driver, m_testing, drivetrain, m_intake, m_shooter, m_indexer, m_feeder);

    m_leds = new Leds(m_superstructure);

    // Fuel Simulation
    if (Mode.currentMode == CurrentMode.SIM) {
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
          width / 2 + Units.inchesToMeters(10),
          -length / 2,
          length / 2,
          () ->
              m_driver.leftTrigger().getAsBoolean()
                  || m_superstructure.getCurrentState() == CurrentStates.IntakeAuto);

      fuelSim.setSubticks(5);

      fuelSim.start();

      fuelSim.enableAirResistance();

      fuelSim.shouldShoot =
          () ->
              (m_driver.rightTrigger().getAsBoolean()
                      || m_superstructure.getCurrentState() == CurrentStates.ScoreAuto)
                  && DriverStation.isEnabled();
      fuelSim.shouldScore = () -> !m_superstructure.shooting();

      m_shooter.setFuelSim(fuelSim);
    }

    m_autoChooser = new AutosChooser(m_superstructure, drivetrain, m_shooter, m_intake);

    configureBindings();
  }

  public void setTeleCurrentLimits() {
    drivetrain.setTeleCurrentLimits();
  }

  /** Adds testing-specific button bindings for subsystem control. */
  public void addTestingBindings() {
    m_testing
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.Intaking)))
        .onFalse(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.Down)));

    m_testing
        .a()
        .onTrue(Commands.runOnce(() -> m_indexer.setStateTesting(IndexStates.Indexing)))
        .onFalse(Commands.runOnce(() -> m_indexer.setStateTesting(IndexStates.Idle)));

    m_testing
        .b()
        .onTrue(Commands.runOnce(() -> m_feeder.setStateTesting(FEEDER_STATE.FEEDING)))
        .onFalse(Commands.runOnce(() -> m_feeder.setStateTesting(FEEDER_STATE.STOP)));

    m_testing
        .x()
        .onTrue(Commands.runOnce(() -> m_shooter.setStateTesting(SHOOTER_STATE.CORNER)))
        .onFalse(Commands.runOnce(() -> m_shooter.setStateTesting(SHOOTER_STATE.IDLE)));

    m_testing
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_shooter.setStateTesting(SHOOTER_STATE.TEST);
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

    m_testing.povRight().onTrue(Commands.runOnce(() -> m_intake.setStateTesting(IntakeStates.Up)));
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

    m_driver
        .rightStick()
        .and(m_driver.leftStick())
        .onTrue(
            Commands.runOnce(() -> drivetrain.setShouldAcceptNextVisionMeasurementRotation(true))
                .ignoringDisable(true));

    m_driver
        .rightTrigger()
        .onTrue(m_superstructure.setIntakeAddableStateCommand(IntakeAddableStates.Intaking));

    m_driver
        .rightTrigger()
        .and(m_superstructure::driveAtTarget)
        .and(() -> m_hasntAcceptedVisionRotation)
        .onTrue(
            Commands.runOnce(() -> drivetrain.setShouldAcceptNextVisionMeasurementRotation(true))
                .andThen(Commands.runOnce(() -> m_hasntAcceptedVisionRotation = false))
                .ignoringDisable(true));

    m_driver
        .leftStick()
        .and(() -> DriverStation.isTeleopEnabled())
        .onTrue(
            Commands.runOnce(
                () -> {
                  DrivetrainAutomationConstants.BumpDetection.toggleAutoBumpAlignment();
                  DrivetrainAccelerationLimits.toggleShouldLimit();
                }));

    m_driver
        .start()
        .onTrue(
            Commands.runOnce(() -> drivetrain.resetRotation(Rotation2d.kZero))
                .ignoringDisable(true));

    m_driver
        .rightTrigger()
        .and(m_driver.leftTrigger().negate())
        .onTrue(m_superstructure.setWantedStateCommand(WantedStates.Shoot));

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

    m_driver.rightTrigger().onFalse(Commands.runOnce(() -> m_hasntAcceptedVisionRotation = true));

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
        .onTrue(m_superstructure.setIntakeAddableStateCommand(IntakeAddableStates.IntakeUp));

    m_driver
        .povRight()
        .and(m_superstructure::currentStateUsesIntake)
        .onFalse(m_superstructure.setIntakeAddableStateCommand(IntakeAddableStates.Intaking));

    new Trigger(DriverStation::isTeleopEnabled)
        .onTrue(
            Commands.sequence(
                Commands.waitSeconds(3.5),
                Commands.runOnce(() -> m_driver.setRumble(RumbleType.kBothRumble, 1)),
                Commands.waitSeconds(1.5),
                Commands.runOnce(() -> m_driver.setRumble(RumbleType.kBothRumble, 0))));
  }

  /** Returns the autonomous command to run during autonomous period. */
  public Command getAutonomousCommand() {
    return m_autoChooser.selectAuto();
  }
}
