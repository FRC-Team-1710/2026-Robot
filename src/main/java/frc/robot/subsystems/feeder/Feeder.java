package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Feeder {
  private FEEDER_STATE m_state;
  private final FeederIO m_io;

  private final TimesConsumer m_timesConsumer;

  public Feeder(FeederIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_state = FEEDER_STATE.STOP;
  }

  public void periodic() {
    switch (this.m_state) {
      case STOP:
        this.m_io.stop();
      case FEEDDING:
        this.m_io.setVelocity(true);
      case REVERSE:
        this.m_io.setVelocity(false);
    }

    this.m_io.update();
  }

  public enum FEEDER_STATE {
    STOP(Milliseconds.of(60), RotationsPerSecond.of(0)),
    FEEDDING(Milliseconds.of(60), RotationsPerSecond.of(10)),
    REVERSE(Milliseconds.of(60), RotationsPerSecond.of(-10));

    private final Time m_subsystemPeriodicFrequency;
    private final AngularVelocity m_velocity;

    FEEDER_STATE(Time pSubsystemPeriodicFrequency, AngularVelocity pVelocity) {
      this.m_subsystemPeriodicFrequency = pSubsystemPeriodicFrequency;
      this.m_velocity = pVelocity;
    }

    Time getSubsystemPeriodicFrequency() {
      return this.m_subsystemPeriodicFrequency;
    }

    AngularVelocity getVelocity() {
      return this.m_velocity;
    }
  }

  public void setState(FEEDER_STATE state) {
    if (!this.m_state
        .getSubsystemPeriodicFrequency()
        .isEquivalent(state.getSubsystemPeriodicFrequency())) {
      m_timesConsumer.accept(Subsystems.Feeder, state.getSubsystemPeriodicFrequency());
    }
    this.m_state = state;
  }

  public FEEDER_STATE getState() {
    return this.m_state;
  }
}
