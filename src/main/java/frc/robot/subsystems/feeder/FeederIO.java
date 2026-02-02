package frc.robot.subsystems.feeder;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface FeederIO {
  public default void update() {}

  public default void setVelocity(boolean clockwise) {}

  public default void stop() {}
}
