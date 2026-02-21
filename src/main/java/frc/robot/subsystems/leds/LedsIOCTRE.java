package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.SerialPort;

@Logged
@SuppressWarnings("unused")
public class LedsIOCTRE implements LedsIO {
  private int m_data;

  private final SerialPort m_port;

  public LedsIOCTRE() {
    this.m_data = 0;

    this.m_port = new SerialPort(115200, SerialPort.Port.kMXP);
    // this.m_bits =
    // new DigitalOutput[] {m_johnRomero, m_johnCarmack, m_adrianCarmack, m_sandyPetersen};
  }

  public void update() {
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

  public void setFlyWheelCharge(double pPercentage) {
    int formatedPercent = (int) (30 * pPercentage);
    for (int i = 0; i < 5; i++) {
      this.m_data = formatedPercent | (1 << i);
    }
  }

  public void send() {
    byte[] data = new byte[1];
    data[0] = (byte) (this.m_data & 0xFF);

    this.m_port.write(data, data.length);
  }
}
