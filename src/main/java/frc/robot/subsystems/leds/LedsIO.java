package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface LedsIO {
  public default void update() {}

  public enum LED_STATE {
    AUTOS,
    BROWOUT,
    ATTACKING
  };

  public default void setValue(LED_STATE pState, boolean pValue) {}

  public default void setFlyWheelCharge(double pPercentage) {}

  public default void send() {}
}
