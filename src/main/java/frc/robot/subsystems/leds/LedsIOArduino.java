package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.SerialPort;

@Logged
@SuppressWarnings("unused")
public class LedsIOArduino implements LedsIO {
  private int m_data;

  private final SerialPort m_port;

  public LedsIOArduino() {
    this.m_data = 0;

    this.m_port = new SerialPort(115200, SerialPort.Port.kMXP);
  }

  /** {@inheritDoc} */
  public void update() {
    byte[] data = new byte[1];
    data[0] = (byte) (this.m_data & 0xFF);

    this.m_port.write(data, data.length);
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
