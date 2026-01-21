package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class Mode {
  public static final CurrentMode currentMode =
      RobotBase.isSimulation() ? CurrentMode.SIMULATION : CurrentMode.REAL;

  public enum CurrentMode {
    SIMULATION,
    REAL
  }
}
