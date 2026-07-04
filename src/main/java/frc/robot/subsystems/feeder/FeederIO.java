package frc.robot.subsystems.feeder;

public interface FeederIO {
  public default void update(double dtSeconds) {}

  public default void setFeeder(double percent) {}
}
