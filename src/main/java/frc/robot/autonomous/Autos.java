// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;

/** Add your docs here. */
public class Autos {
  private final HashMap<Auto, Command> autoCommands;
  private final SendableChooser<Auto> autoChooser;

  public Autos() {
    autoCommands = new HashMap<>();
    autoCommands.put(Auto.NONE, Commands.none());
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", Auto.NONE);
  }

  public void addPath(Auto auto, Command command) {
    autoCommands.put(auto, command);
    autoChooser.addOption(auto.name(), auto);
  }

  public Command getAuto() {
    return autoCommands.get(autoChooser.getSelected());
  }

  public enum Auto {
    NONE(),
    TEST_PATH()
  }
}
