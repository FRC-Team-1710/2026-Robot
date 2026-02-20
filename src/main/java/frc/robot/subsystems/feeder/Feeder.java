package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Subsystems;
import frc.robot.utils.DynamicTimedRobot.TimesConsumer;

@Logged
public class Feeder {
  @Logged(importance = Importance.CRITICAL)
  private FEEDER_STATE m_currentState;

  @Logged(importance = Importance.CRITICAL)
  private final FeederIO m_io;

  @NotLogged private final TimesConsumer m_timesConsumer;

  public Feeder(FeederIO io, TimesConsumer consumer) {
    this.m_io = io;
    this.m_timesConsumer = consumer;
    this.m_currentState = FEEDER_STATE.STOP;
  }

  public void periodic() {
    this.m_io.setFeeder(this.m_currentState.m_velocity);

    this.m_io.update(m_currentState.getSubsystemPeriodicFrequency().in(Seconds));
  }

  public enum FEEDER_STATE {
    STOP(Milliseconds.of(60), 0),
    FEEDING(Milliseconds.of(20), 0.5),
    REVERSE(Milliseconds.of(20), -0.25);

    private final Time m_subsystemPeriodicFrequency;
    private final double m_velocity;

    FEEDER_STATE(Time pSubsystemPeriodicFrequency, double pVelocity) {
      this.m_subsystemPeriodicFrequency = pSubsystemPeriodicFrequency;
      this.m_velocity = pVelocity;
    }

    Time getSubsystemPeriodicFrequency() {
      return this.m_subsystemPeriodicFrequency;
    }

    double getVelocity() {
      return this.m_velocity;
    }
  }

  public void setState(FEEDER_STATE state) {
    if (!this.m_currentState
        .getSubsystemPeriodicFrequency()
        .isEquivalent(state.getSubsystemPeriodicFrequency())) {
      m_timesConsumer.accept(Subsystems.Feeder, state.getSubsystemPeriodicFrequency());
    }
    this.m_currentState = state;
  }

  @NotLogged
  public FEEDER_STATE getState() {
    return this.m_currentState;
  }
}
