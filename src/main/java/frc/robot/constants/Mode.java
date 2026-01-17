package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public enum Mode {
  SIMULATION,
  REAL;

  public static Mode getCurrentMode() {
    return RobotBase.isReal() ? Mode.REAL : Mode.SIMULATION;
  }
}
