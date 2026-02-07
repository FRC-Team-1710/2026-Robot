package frc.robot.subsystems.feeder;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface FeederIO {
  public default void update() {}

  public default void setFeeder(double percent) {}
}
