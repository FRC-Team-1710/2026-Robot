package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class Mode {
  private static final CurrentMode kSimMode = CurrentMode.SIM;

  public static final CurrentMode currentMode =
      RobotBase.isSimulation() ? kSimMode : CurrentMode.REAL;

  public enum CurrentMode {
    REAL,
    SIM,
    REPLAY
  }
}
