package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface LedsIO {
  public default void update(double pDTSeconds) {}

  public default void send() {}
}
