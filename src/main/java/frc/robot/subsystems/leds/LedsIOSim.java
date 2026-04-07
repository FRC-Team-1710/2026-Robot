package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import frc.robot.Robot;

@Logged
@SuppressWarnings("unused")
public class LedsIOSim implements LedsIO {
  private int m_data;

  public LedsIOSim() {
    this.m_data = 0;
  }

  /** {@inheritDoc} */
  public void update() {
    boolean[] array = new boolean[8];

    for (int i = 0; i < 8; i++) {
      boolean value = ((this.m_data >> i) & 1) == 1;
      array[i] = value;
    }

    Robot.telemetry().log("Led Bits", array);
  }


  /** {@inheritDoc} */
  public void resetValue() {
    this.m_data = 0;
  }
  
  /** {@inheritDoc} */
  public void setValue(LED_STATE pState, boolean pValue) {
    int offset = getStateOffset(pState);

    if (pValue) {
      this.m_data |= (1 << offset);
    } else {
      this.m_data &= ~(1 << offset);
    }
  }

  /** {@inheritDoc} */
  private int getStateOffset(LED_STATE pState) {
    switch (pState) {
      case DISBALED:
        return 0;
      case IN_AUTOS:
        return 1;
      case AUTOS_VICTORY:
        return 2;
      case BROWNOUT:
        return 3;
      case INTAKING:
        return 4;
      case SHOOTING:
        return 5;
      case CAN_SHOOT:
        return 6;
    }

    return 0;
  }
}
