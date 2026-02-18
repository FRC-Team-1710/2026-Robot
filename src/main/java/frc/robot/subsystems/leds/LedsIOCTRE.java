package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalOutput;

@Logged
@SuppressWarnings("unused")
public class LedsIOCTRE implements LedsIO {
  private final DigitalOutput JohnRomero; // Bit 1 (1)
  private final DigitalOutput JohnCarmack; // Bit 2 (2)
  private final DigitalOutput AdrianCarmack; // Bit 3 (4)
  private final DigitalOutput SandyPetersen; // Bit 4 (8)

  public LedsIOCTRE() {
    this.JohnRomero = new DigitalOutput(3);
    this.JohnCarmack = new DigitalOutput(4);
    this.AdrianCarmack = new DigitalOutput(5);
    this.SandyPetersen = new DigitalOutput(6);
  }

  public void update(double pDTSeconds) {}
}
