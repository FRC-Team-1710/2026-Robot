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
import frc.robot.constants.Mode;
import frc.robot.constants.Subsystems;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOCTRE;
import frc.robot.subsystems.intake.IntakeIOSIM;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOCTRE;
import frc.robot.subsystems.shooter.ShooterIOSIM;

import java.util.HashMap;

@Logged
@SuppressWarnings("unused")
public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController mech = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain;

  /* Create subsystems (uses simulated versions when running in simulation) */
  private final Intake intake;
  private final Shooter shooter;

  public RobotContainer() {
    drivetrain = TunerConstants.createDrivetrain();
    drivetrain.setController(driver);

    switch (Mode.currentMode) {
      case REAL:
        intake = new Intake(new IntakeIOCTRE());
        shooter = new Shooter(new ShooterIOCTRE());
        break;

      case SIMULATION:
        intake = new Intake(new IntakeIOSIM());
        shooter = new Shooter(new ShooterIOSIM());
        break;

      default:
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    superstructure = new Superstructure(driver, mech, drivetrain, intake);

    configureBindings();
  }

  private void configureBindings() {
    driver
        .start()
        .onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(Feet.of(0), Feet.of(0), Rotation2d.kZero))));

    driver
        .povRight()
        .onTrue(superstructure.setWantedStateCommand(WantedStates.AssistRight))
        .onFalse(superstructure.setWantedStateCommand(WantedStates.Default));

    driver
        .povLeft()
        .onTrue(superstructure.setWantedStateCommand(WantedStates.AssistLeft))
        .onFalse(superstructure.setWantedStateCommand(WantedStates.Default));

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
    return drivetrain.getAuto();
  }

  public HashMap<Subsystems, Pair<Runnable, Time>> getAllSubsystems() {
    HashMap<Subsystems, Pair<Runnable, Time>> map = new HashMap<>();
    map.put(
        Subsystems.Superstructure,
        new Pair<Runnable, Time>(superstructure::periodic, Milliseconds.of(20)));
    map.put(Subsystems.Drive, new Pair<Runnable, Time>(drivetrain::periodic, Milliseconds.of(20)));
    return map;
  }
}
