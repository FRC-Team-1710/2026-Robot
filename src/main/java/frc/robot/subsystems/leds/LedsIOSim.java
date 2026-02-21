package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import frc.robot.Robot;

@Logged
@SuppressWarnings("unused")
public class LedsIOSIM implements LedsIO {
  private int m_data;

  public LedsIOSIM() {
    this.m_data = 0;
  }

  public void update(double pDTSeconds) {
    this.encode();
    this.send();
  }

  public void setValue(LED_STATE pState, boolean pValue) {
    int offset = 0;
    switch (pState) {
      case AUTOS:
        offset = 8;
        break;

      case BROWOUT:
        offset = 7;
        break;

      case ATTACKING:
        offset = 6;
        break;
    }
    this.m_data = this.m_data | (1 << offset);
  }

  public void setFlyWheelCharge(int pPercentage) {
    for (int i = 0; i < 5; i++) {
      this.m_data = pPercentage | (1 << i);
    }
  }

  public void send() {
    boolean[] array = new boolean[8];

    for (int i = 0; i < 8; i++) {
      boolean value = ((this.m_data >> i) & 1) == 1;
      array[i] = value;
    }

    Robot.telemetry().log("Led Bits", array);
  }
}
