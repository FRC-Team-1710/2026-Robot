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
import frc.robot.Robot;
import frc.robot.constants.Subsystems;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.PriorityQueue;

/**
 * DynamicTimedRobot was HEAVILY inspired by TimedRobot which implements the IterativeRobotBase
 * robot program framework.
 *
 * <p>It pretty much uses the same callback and PriorityQueue system but instead with options to
 * adjust the periodic rate of the subsystems, hence the name DynamicTimedRobot
 */
public class DynamicTimedRobot extends IterativeRobotBase {
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
      return rhs instanceof Callback callback
          && period == callback.period
          && subsystem == callback.subsystem;
    }

    @Override
    public int hashCode() {
      return Long.hashCode(expirationTime);
    }

    @Override
    public int compareTo(Callback rhs) {
      // Elements with sooner expiration times are sorted as lesser. The head of
      // Java's PriorityQueue is the least element.
      return Long.compare(expirationTime, rhs.expirationTime);
    }
  }

  /** Default loop period (20ms) */
  public static final Time kDefaultPeriod = Seconds.of(0.02);

  // The C pointer to the notifier object. We don't use it directly, it is
  // just passed to the JNI bindings.
  private final int m_notifier = NotifierJNI.initializeNotifier();

  private long m_startTimeUs;
  private long m_loopStartTimeUs;
  private long previousStartOfPeriodic;

  private long totalCodeTime = 0;

  private long currentTime = 0;

  private Callback currentPeriodicCallback;

  private final PriorityQueue<Callback> m_callbacks = new PriorityQueue<>();

  private final HashMap<Subsystems, Runnable> subsystemToRunnable = new HashMap<>();

  private final HashMap<Subsystems, Long> previousSubsystemTimes = new HashMap<>();

  private final ArrayList<Subsystems> runImmediately = new ArrayList<>();

  private ArrayList<String> subsystemsRunThisLoop = new ArrayList<>();

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
    previousStartOfPeriodic = m_loopStartTimeUs;
    addSubsystem(Subsystems.Robot, this::loopFunc, period);
    NotifierJNI.setNotifierName(m_notifier, "TimedRobot");

    HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Timed);
  }

  @Override
  public void close() {
    NotifierJNI.stopNotifier(m_notifier);
    NotifierJNI.cleanNotifier(m_notifier);
  }

  /** Provide an alternate "main loop" via startCompetition(). */
  @Override
  public void startCompetition() {
    robotInit();

    if (isSimulation()) {
      simulationInit();
    }

    // Tell the DS that the robot is ready to be enabled
    System.out.println("********** Robot program startup complete **********");
    DriverStationJNI.observeUserProgramStarting();

    // Loop forever, calling the appropriate mode-dependent function
    while (true) {
      // We don't have to check there's an element in the queue first because
      // there's always at least one (the constructor adds one). It's reenqueued
      // at the end of the loop.

      var callback = m_callbacks.poll();

      NotifierJNI.updateNotifierAlarm(m_notifier, callback.expirationTime);

      currentTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
      if (currentTime == 0) {
        break;
      }

      runPeriodic(callback);

      m_callbacks.add(callback);

      for (Object objectSubsystemToRun : runImmediately.toArray()) {
        Subsystems subsystemToRun = (Subsystems) objectSubsystemToRun;
        for (Object obj : m_callbacks.toArray()) {
          Callback tempCallback = (Callback) obj;
          // If the callback were looking for is found
          if (tempCallback.subsystem == subsystemToRun) {
            m_callbacks.remove(tempCallback);
            runPeriodic(tempCallback);
            m_callbacks.add(tempCallback);
            System.out.println(subsystemToRun);
            break;
          }
        }
        runImmediately.remove(subsystemToRun);
      }

      // Process all other callbacks that are ready to run
      while (m_callbacks.peek().expirationTime <= currentTime) {
        callback = m_callbacks.poll();

        currentPeriodicCallback = callback;

        runPeriodic(callback);

        m_callbacks.add(callback);

        for (Object objectSubsystemToRun : runImmediately.toArray()) {
          Subsystems subsystemToRun = (Subsystems) objectSubsystemToRun;
          for (Object obj : m_callbacks.toArray()) {
            Callback tempCallback = (Callback) obj;
            // If the callback were looking for is found
            if (tempCallback.subsystem == subsystemToRun) {
              m_callbacks.remove(tempCallback);
              runPeriodic(tempCallback);
              m_callbacks.add(tempCallback);
              System.out.println(subsystemToRun);
              break;
            }
          }
          runImmediately.remove(subsystemToRun);
        }
      }
    }
  }

  private void runPeriodic(Callback callback) {
    Robot.telemetry()
        .log(
            "Periodics/" + callback.subsystem.toString() + "/TimeBetweenTriggers",
            RobotController.getFPGATime() - previousSubsystemTimes.get(callback.subsystem));
    previousSubsystemTimes.put(callback.subsystem, RobotController.getFPGATime());

    if (callback.subsystem == Subsystems.Robot) {
      Robot.telemetry()
          .log("Periodics/Total", RobotController.getFPGATime() - previousStartOfPeriodic);
      previousStartOfPeriodic = RobotController.getFPGATime();
      Robot.telemetry().log("Periodics/TotalCode", totalCodeTime);
      totalCodeTime = 0;
      Robot.telemetry().log("Periodics/SubsystemsRunThisLoop/Size", subsystemsRunThisLoop.size());
      Robot.telemetry()
          .log("Periodics/SubsystemsRunThisLoop/Values", subsystemsRunThisLoop.toString());
      subsystemsRunThisLoop = new ArrayList<>();
    }

    var tempTime = RobotController.getFPGATime();

    callback.func.run();

    totalCodeTime += RobotController.getFPGATime() - tempTime;

    Robot.telemetry()
        .log(
            "Periodics/" + callback.subsystem.toString() + "/Periodic",
            RobotController.getFPGATime() - tempTime);

    subsystemsRunThisLoop.add(callback.subsystem.toString());

    callback.expirationTime +=
        callback.period
            + (currentTime - callback.expirationTime) / callback.period * callback.period;
  }

  /** Ends the main loop in startCompetition(). */
  @Override
  public void endCompetition() {
    NotifierJNI.stopNotifier(m_notifier);
  }

  /**
   * Return the system clock time in microseconds for the start of the current periodic loop. This
   * is in the same time base as Timer.getFPGATimestamp(), but is stable through a loop. It is
   * updated at the beginning of every periodic callback (including the normal periodic loop).
   *
   * @return Robot running time in microseconds, as of the start of the current periodic function.
   */
  public long getLoopStartTime() {
    return m_loopStartTimeUs;
  }

  /** Returns a new callback with the given params */
  private Callback getCallback(Subsystems subsystem, Runnable periodic, Time period, Time offset) {
    return new Callback(
        periodic,
        m_startTimeUs,
        (long) (period.in(Seconds) * 1e6),
        (long) (offset.in(Seconds) * 1e6),
        subsystem);
  }

  /**
   * Adds a subsystem to the queue of runnables
   *
   * <p>Assumes an offset of zero (default)
   *
   * @param subsystem Subsystem to add (enum in constants)
   * @param periodic Subsystem periodic function as a Runnable
   * @param period How frequently to call periodic
   */
  public final void addSubsystem(Subsystems subsystem, Runnable periodic, Time period) {
    addSubsystem(subsystem, periodic, period, Seconds.of(0));
  }

  /**
   * Adds a subsystem to the que of runnables
   *
   * @param subsystem Subsystem to add (enum in constants)
   * @param periodic Subsystem periodic function as a Runnable
   * @param period How frequently to call periodic
   * @param offset Offset relative to main loop
   */
  public final void addSubsystem(
      Subsystems subsystem, Runnable periodic, Time period, Time offset) {
    subsystemToRunnable.put(subsystem, periodic);
    m_callbacks.add(getCallback(subsystem, periodic, period, offset));
    previousSubsystemTimes.put(subsystem, RobotController.getFPGATime());
  }

  /**
   * Set new period for subsystem
   *
   * <p>This function ensures the offset set originally isn't change and that it will only start
   * running at the new period
   *
   * @param subsystem Subsystem to change (enum in constants)
   * @param period How frequently to call periodic
   */
  public final void setSubsystem(Subsystems subsystem, Time period) {
    if (currentPeriodicCallback != null && currentPeriodicCallback.subsystem != subsystem) {
      for (Object obj : m_callbacks.toArray()) {
        Callback callback = (Callback) obj;
        // If the callback were looking for is found
        if (callback.subsystem == subsystem) {
          callback.period = (long) (period.in(Seconds) * 1e6);
          break;
        }
      }
    } else {
      currentPeriodicCallback.period = (long) (period.in(Seconds) * 1e6);
    }
  }

  /**
   * The consumer of new periods and offsets for subsystems
   *
   * @param subsystem Subsystem to add (enum in constants)
   * @param period How frequently to call periodic
   */
  public void setSubsystemConsumer(Subsystems subsystem, Time period) {
    setSubsystem(subsystem, period);
    runImmediately.add(subsystem);
  }

  @FunctionalInterface
  public static interface TimesConsumer {
    void accept(Subsystems subsystem, Time period);
  }
}
