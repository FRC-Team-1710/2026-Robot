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

  public void update() {
    byte[] data = new byte[1];
    data[0] = (byte) (this.m_data & 0xFF);

    this.m_port.write(data, data.length);
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

    int bitValue = pValue ? 0 : 1;
    this.m_data = bitValue | (1 << offset);
  }

  public void setFlyWheelCharge(double pPercentage) {
    int formatedPercent = (int) (30 * pPercentage);
    for (int i = 0; i < 5; i++) {
      this.m_data = formatedPercent | (1 << i);
    }
  }
}
