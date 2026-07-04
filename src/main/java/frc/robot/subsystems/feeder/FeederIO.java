package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederInputs {
    public double motorVelocity = 0.0;
    public double motorCurrent = 0.0;
  }

  /**
   * Updates the set of logged inputs for the feeder subsystem.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(FeederInputs inputs) {}

  public default void update(double dtSeconds) {}

  public default void setFeeder(double percent) {}
}