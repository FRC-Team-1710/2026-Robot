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
  public void setValue(LED_STATE pState, boolean pValue) {
    int offset = 0;
    switch (pState) {
      case AUTOS:
        offset = 7;
        break;

      case BROWNOUT:
        offset = 6;
        break;

      case ATTACKING:
        offset = 5;
        break;
    }

    if (pValue) {
      this.m_data |= (1 << offset);
    } else {
      this.m_data &= ~(1 << offset);
    }
  }

  /** {@inheritDoc} */
  public void setFlyWheelCharge(double pPercentage) {
    int formattedPercent = (int) (30 * pPercentage);
    for (int i = 0; i < 5; i++) {
      this.m_data = formattedPercent | (1 << i);
    }
  }
}
