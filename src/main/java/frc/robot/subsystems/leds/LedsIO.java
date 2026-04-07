package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;

@Logged
public interface LedsIO {
  public default void update() {}

  public enum LED_STATE {
    DISBALED,
    IN_AUTOS,
    AUTOS_VICTORY,
    BROWNOUT,
    INTAKING,
    SHOOTING,
    CAN_SHOOT
  };

  public default void resetValue() {}
  public default void setValue(LED_STATE pState, boolean pValue) {}
}
