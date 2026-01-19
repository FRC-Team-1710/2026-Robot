// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class Autos {

  public enum Auto {
    TEST_PATH(Commands.sequence(null)),
    EAT_BURGER(Commands.sequence(null));

    private final Command command;

    Auto(Command command) {
      this.command = command;
    }

    public Command getCommand() {
      return command;
    }
  }
}
