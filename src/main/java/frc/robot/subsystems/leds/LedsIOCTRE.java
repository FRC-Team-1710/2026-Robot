package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@Logged
public class LedsIOCTRE implements LedsIO {
  private boolean[] m_output;

  private final DigitalOutput m_johnRomero; // Bit 1 (1)
  private final DigitalOutput m_johnCarmack; // Bit 2 (2)
  private final DigitalOutput m_adrianCarmack; // Bit 3 (4)
  private final DigitalOutput m_sandyPetersen; // Bit 4 (8)

  public final DigitalOutput[] m_bits;

  public LedsIOCTRE() {
    this.m_output = new boolean[4];

    this.m_johnRomero = new DigitalOutput(3);
    this.m_johnCarmack = new DigitalOutput(4);
    this.m_adrianCarmack = new DigitalOutput(5);
    this.m_sandyPetersen = new DigitalOutput(6);

    this.m_bits =
        new DigitalOutput[] {m_johnRomero, m_johnCarmack, m_adrianCarmack, m_sandyPetersen};
  }

  public void update(double pDTSeconds) {}

  public void send() {
    for (int i = 0; i < this.m_output.length; i++) {
      this.m_bits[i].set(this.m_output[i]);
    }

    SmartDashboard.putBooleanArray("DigitalOutputs", this.m_output);
  }
}
