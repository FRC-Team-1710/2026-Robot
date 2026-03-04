package frc.robot.subsystems.feeder;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface FeederIO {
  public default void update(double dtSeconds) {}

  public default void setLeft(double percent) {}

  public default void setRight(double percent) {}
}
