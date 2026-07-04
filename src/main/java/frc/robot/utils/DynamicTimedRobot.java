// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.Subsystems;
import java.util.HashMap;
import java.util.PriorityQueue;
import org.littletonrobotics.junction.Logger;

/**
 * DynamicTimedRobot was HEAVILY inspired by TimedRobot which implements the IterativeRobotBase
 * robot program framework.
 *
 * <p>It pretty much uses the same callback and PriorityQueue system but instead with options to
 * adjust the periodic rate of the subsystems, hence the name DynamicTimedRobot
 */
public class DynamicTimedRobot extends IterativeRobotBase {
  public static record SubsystemInfo(
      Subsystems subsystem, Runnable periodic, Time period, Time offset) {}

  /**
   * A container that contains the Runnable function, rate (period), expiration time (not set by
   * user), and Subsystem
   */
  static class Callback implements Comparable<Callback> {
    public Runnable func;
    public long period;
    public long expirationTime;
    public Subsystems subsystem;

    /**
     * Construct a callback container.
     *
     * @param func The Runnable to run.
     * @param startTimeUs The common starting point for all callback scheduling in microseconds.
     * @param periodUs The period at which to run the callback in microseconds.
     * @param offsetUs The offset from the common starting time in microseconds.
     */
    Callback(Runnable func, long startTimeUs, long periodUs, long offsetUs, Subsystems subsystem) {
      this.func = func;
      this.period = periodUs;
      this.expirationTime =
          startTimeUs
              + offsetUs
              + this.period
              + (RobotController.getFPGATime() - startTimeUs) / this.period * this.period;
      this.subsystem = subsystem;
    }

    @Override
    public boolean equals(Object rhs) {
      return rhs instanceof Callback callback && expirationTime == callback.expirationTime;
    }

    @Override
    public int hashCode() {
      return Long.hashCode(expirationTime);
    }

    @Override
    public int compareTo(Callback rhs) {
      return Long.compare(expirationTime, rhs.expirationTime);
    }
  }

  /** Default loop period (20ms) */
  public static final Time kDefaultPeriod = Seconds.of(0.02);

  private final int m_notifier = NotifierJNI.initializeNotifier();

  private long m_startTimeUs;
  private long m_loopStartTimeUs;
  private long m_previousStartOfPeriodic;

  private long m_totalCodeTime = 0;

  private long m_currentTime = 0;

  private Callback m_currentPeriodicCallback;

  private final PriorityQueue<Callback> m_callbacks = new PriorityQueue<>();

  private final HashMap<Subsystems, Callback> m_subsystemToCallback = new HashMap<>();

  /** Constructor for DynamicTimedRobot. */
  protected DynamicTimedRobot() {
    this(kDefaultPeriod);
  }

  /**
   * Constructor for DynamicTimedRobot.
   *
   * @param period Period.
   */
  protected DynamicTimedRobot(Time period) {
    super(period.in(Seconds));
    m_startTimeUs = RobotController.getFPGATime();
    m_previousStartOfPeriodic = m_loopStartTimeUs;
    addSubsystem(new SubsystemInfo(Subsystems.Robot, this::loopFunc, period, Seconds.of(0)));
    NotifierJNI.setNotifierName(m_notifier, "TimedRobot");

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
  }

  @Override
  public void close() {
    NotifierJNI.stopNotifier(m_notifier);
    NotifierJNI.cleanNotifier(m_notifier);
  }

  @Override
  public void startCompetition() {
    robotInit();

    if (isSimulation()) {
      simulationInit();
    }

    System.out.println("********** Robot program startup complete **********");
    DriverStationJNI.observeUserProgramStarting();

    while (true) {
      m_loopStartTimeUs = RobotController.getFPGATime();

      var callback = m_callbacks.poll();

      NotifierJNI.updateNotifierAlarm(m_notifier, callback.expirationTime);

      m_currentTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
      if (m_currentTime == 0) {
        break;
      }

      runPeriodic(callback);

      m_callbacks.add(callback);

      while (m_callbacks.peek().expirationTime <= m_currentTime) {
        callback = m_callbacks.poll();

        m_currentPeriodicCallback = callback;

        runPeriodic(callback);

        m_callbacks.add(callback);
      }
    }
  }

  private void runPeriodic(Callback callback) {
    if (callback.subsystem == Subsystems.Robot) {
      Logger.recordOutput("Periodics/Total", RobotController.getFPGATime() - m_previousStartOfPeriodic);
      m_previousStartOfPeriodic = RobotController.getFPGATime();
      Logger.recordOutput("Periodics/TotalCode", m_totalCodeTime);
      m_totalCodeTime = 0;
    }

    var tempTime = RobotController.getFPGATime();

    callback.func.run();

    m_totalCodeTime += RobotController.getFPGATime() - tempTime;

    Logger.recordOutput(
        "Periodics/" + callback.subsystem.toString() + "/Periodic",
        RobotController.getFPGATime() - tempTime);

    callback.expirationTime +=
        callback.period
            + (m_currentTime - callback.expirationTime) / callback.period * callback.period;
  }

  @Override
  public void endCompetition() {
    NotifierJNI.stopNotifier(m_notifier);
  }

  public long getLoopStartTime() {
    return m_loopStartTimeUs;
  }

  private Callback getCallback(Subsystems subsystem, Runnable periodic, Time period, Time offset) {
    return new Callback(
        periodic,
        m_startTimeUs,
        (long) (period.in(Seconds) * 1e6),
        (long) (offset.in(Seconds) * 1e6),
        subsystem);
  }

  public final void addSubsystem(SubsystemInfo subsystemInfo) {
    var callback =
        getCallback(
            subsystemInfo.subsystem,
            subsystemInfo.periodic,
            subsystemInfo.period,
            subsystemInfo.offset);
    m_subsystemToCallback.put(subsystemInfo.subsystem, callback);
    m_callbacks.add(callback);
  }

  public void addAllSubsystems(SubsystemInfo[] subsystemsInfo) {
    for (SubsystemInfo subsystemInfo : subsystemsInfo) {
      addSubsystem(subsystemInfo);
    }
  }

  public final void setSubsystem(Subsystems subsystem, Time period) {
    if (m_currentPeriodicCallback.subsystem != subsystem) {
      var callback = m_subsystemToCallback.get(subsystem);
      callback.expirationTime -= callback.period;
      callback.period = (long) (period.in(Seconds) * 1e6);
      callback.expirationTime += callback.period;
    } else {
      m_currentPeriodicCallback.period = (long) (period.in(Seconds) * 1e6);
    }
  }

  public void setSubsystemConsumer(Subsystems subsystem, Time period) {
    setSubsystem(subsystem, period);
  }

  @FunctionalInterface
  public static interface TimesConsumer {
    void accept(Subsystems subsystem, Time period);
  }
}