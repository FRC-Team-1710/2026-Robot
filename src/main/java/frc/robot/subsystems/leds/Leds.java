package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;

@Logged
public class Leds {
  private final LedsIO m_io;

  public Leds(LedsIO pIo) {
    this.m_io = pIo;
  }

  public void periodic() {
    this.m_io.update();
  }
}
