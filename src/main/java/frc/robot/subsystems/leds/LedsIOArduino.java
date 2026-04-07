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

    this.m_port = new SerialPort(115200, SerialPort.Port.kUSB);
  }

  /** {@inheritDoc} */
  public void update() {
    byte[] data = new byte[1];
    data[0] = (byte) (this.m_data & 0xFF);

    System.out.println(data[0]);
    this.m_port.write(data, data.length);
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
