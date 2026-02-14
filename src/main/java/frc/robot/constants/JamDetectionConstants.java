package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class JamDetectionConstants {
  public class Intake {
    /** 750rpm */
    public static final AngularVelocity kJamSpeedThreshold = RotationsPerSecond.of(12.5);

    public static final Current kJamCurrent = Amps.of(45);

    /** Jam trip time */
    public static final Time kJamMinimumTime = Seconds.of(0.5);

    /** Time from when the intake starts to when it starts detecting jams */
    public static final Time kJamDetectionDisabledTime = Seconds.of(0.5);

    /** Time from when the intake is correcting the jam to when it should stop */
    public static final Time kJamUndoTime = Seconds.of(0.5);
  }

  public class Indexer {
    /** 750rpm */
    public static final AngularVelocity kJamSpeedThreshold = RotationsPerSecond.of(12.5);

    public static final Current kJamCurrent = Amps.of(45);

    /** Jam trip time */
    public static final Time kJamMinimumTime = Seconds.of(0.5);

    /** Time from when the intake starts to when it starts detecting jams */
    public static final Time kJamDetectionDisabledTime = Seconds.of(0.5);

    /** Time from when the intake is correcting the jam to when it should stop */
    public static final Time kJamUndoTime = Seconds.of(0.5);
  }
}
