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

  public void update() {
    boolean[] array = new boolean[8];

    for (int i = 0; i < 8; i++) {
      boolean value = ((this.m_data >> i) & 1) == 1;
      array[i] = value;
    }

    Robot.telemetry().log("Led Bits", array);
  }

  public void setValue(LED_STATE pState, boolean pValue) {
    int offset = 0;
    switch (pState) {
      case AUTOS:
        offset = 8;
        break;

      case BROWNOUT:
        offset = 7;
        break;

      case ATTACKING:
        offset = 6;
        break;
    }
    this.m_data = this.m_data | (1 << offset);
  }

  public void setFlyWheelCharge(double pPercentage) {
    int formatedPercent = (int) (30 * pPercentage);
    for (int i = 0; i < 5; i++) {
      this.m_data = formatedPercent | (1 << i);
    }
  }
}
