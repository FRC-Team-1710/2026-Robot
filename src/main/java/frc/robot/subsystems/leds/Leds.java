package frc.robot.subsystems.leds;

import edu.wpi.first.epilogue.Logged;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Leds {
  private final LedsIO m_io;

  public Leds(LedsIO pIo, TimesConsumer pConsumer) {
    this.m_io = pIo;
  }

  public void periodic() {}
}
