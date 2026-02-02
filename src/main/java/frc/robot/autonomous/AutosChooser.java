// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Superstructure;
import java.util.HashMap;

/** Add your docs here. */
public class AutosChooser {
  private final HashMap<Auto, Command> autoCommands;
  private final SendableChooser<Auto> autoChooser;

  private final CustomAutoMaker customAutoMaker;

  public AutosChooser(Superstructure superstructure) {
    autoCommands = new HashMap<>();
    autoCommands.put(Auto.NONE, Commands.none());
    autoChooser = new SendableChooser<>();
    SmartDashboard.putData("Auto/AutoChooser", autoChooser);
    autoChooser.setDefaultOption("None", Auto.NONE);
    addPath(Auto.CUSTOM, Commands.none());
    customAutoMaker = new CustomAutoMaker(superstructure);

    // Put preset autos hare//
    addPath(Auto.TEST_PATH, testPath());
    addPath(Auto.MY_LAST_BRAINCELL, myLastBraincell());
  }

  public void setCustom(Command command) {
    autoCommands.put(Auto.CUSTOM, command);
  }

  public void addPath(Auto auto, Command command) {
    autoCommands.put(auto, command);
    autoChooser.addOption(auto.name(), auto);
  }

  public Command getAuto() {
    return autoChooser.getSelected() == Auto.CUSTOM
        ? customAutoMaker.getAuto()
        : autoCommands.get(autoChooser.getSelected());
  }

  public Command myLastBraincell() {
    return Commands.sequence(
        AutoPathBuilder.getBuilder()
            .build(
                new Path(
                    new Path.TranslationTarget(FieldConstants.AutoConstants().get("Bump REn, ")))),
        AutoPathBuilder.getBuilder()
            .build(
                new Path(
                    new Path.TranslationTarget(FieldConstants.AutoConstants().get("Bump REx, ")))),
        AutoPathBuilder.getBuilder()
            .build(
                new Path(
                    new Path.TranslationTarget(FieldConstants.AutoConstants().get("Bump LEx, ")))),
        AutoPathBuilder.getBuilder()
            .build(
                new Path(
                    new Path.TranslationTarget(FieldConstants.AutoConstants().get("Bump LEn, ")))));
  }

  public Command testPath() {
    return Commands.sequence(
        AutoPathBuilder.getBuilder()
            .build(
                new Path(
                    new Path.TranslationTarget(FieldConstants.AutoConstants().get("Bump LEn, ")))));
  }

  public enum Auto {
    NONE(),
    CUSTOM(),
    TEST_PATH(),
    MY_LAST_BRAINCELL()
  }
}
