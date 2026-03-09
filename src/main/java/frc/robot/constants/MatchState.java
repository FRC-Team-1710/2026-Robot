package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class MatchState {
  // TODO: Set to simulate a match environment to test match specific code
  private static final boolean kSimulatePracticeMatch = false;

  /** Used when we want to shoot while our hub is disabled */
  private static boolean m_ignoreFMS = false;

  public static Optional<Boolean> autonomousWinnerIsRed = Optional.empty();

  // Mechanical Advantage's estimate
  public static final double kFuelProcessTime = 1.5;

  // Time after hub inactive until it stops counting
  public static final double kHubCountingEndOffset = 2;

  public static final double kFuelTimeOffset = kFuelProcessTime - kHubCountingEndOffset;

  private static final Timer m_teleopTimer = new Timer();

  public static boolean isActive() {
    return timeTillActive().in(Seconds) <= 0;
  }

  public static void startTeleop() {
    m_teleopTimer.restart();
  }

  public static void setIgnoreFMS(boolean ignoreFMS) {
    m_ignoreFMS = ignoreFMS;
  }

  public static boolean canShoot(double tof) {
    // Default to true
    if (autonomousWinnerIsRed.isEmpty() || !fmsAttached() || m_ignoreFMS) {
      return true;
    }
    return timeTillInactive(tof).in(Seconds) > 0;
  }

  public static final Time timeTillActive() {
    return timeTillActive(0);
  }

  public static final Time timeTillInactive() {
    return timeTillInactive(0);
  }

  public static Time timeTillActive(double tof) {
    if (fmsAttached()) {
      var currentMatchTime = (140 - m_teleopTimer.get()) + kFuelTimeOffset + tof;
      if (DriverStation.isAutonomous() || currentMatchTime <= 30 || currentMatchTime > 130) {
        return Seconds.of(0);
      }
      if (autonomousWinnerIsRed.isPresent()
          && autonomousWinnerIsRed.get() == Alliance.redAlliance) {
        if (currentMatchTime > 105) {
          return Seconds.of(currentMatchTime - 105);
        } else if (currentMatchTime > 80) {
          return Seconds.of(0);
        } else if (currentMatchTime > 55) {
          return Seconds.of(currentMatchTime - 55);
        }
      } else if (autonomousWinnerIsRed.isPresent()
          && autonomousWinnerIsRed.get() != Alliance.redAlliance) {
        if (currentMatchTime > 105) {
          return Seconds.of(0);
        } else if (currentMatchTime > 80) {
          return Seconds.of(currentMatchTime - 80);
        } else if (currentMatchTime > 55) {
          return Seconds.of(0);
        } else if (currentMatchTime > 30) {
          return Seconds.of(currentMatchTime - 30);
        }
      }
    }
    /*
      Already active, no FMS, or unknown auto winner.
      If auto winner is unknown, don't assume an inactive
      hub (rather shoot into inactive than not shoot in active)
    */
    return Seconds.of(0);
  }

  public static Time timeTillInactive(double tof) {
    if (fmsAttached()) {
      var currentMatchTime = (140 - m_teleopTimer.get()) + kFuelTimeOffset + tof;
      if (DriverStation.isAutonomous() || currentMatchTime <= 30) {
        return Seconds.of(0);
      }
      if (autonomousWinnerIsRed.isPresent()
          && autonomousWinnerIsRed.get() != Alliance.redAlliance) {
        if (currentMatchTime > 105) {
          return Seconds.of(currentMatchTime - 105);
        } else if (currentMatchTime > 80) {
          return Seconds.of(0);
        } else if (currentMatchTime > 55) {
          return Seconds.of(currentMatchTime - 55);
        }
      } else if (autonomousWinnerIsRed.isPresent()
          && autonomousWinnerIsRed.get() == Alliance.redAlliance) {
        if (currentMatchTime > 130) {
          return Seconds.of(currentMatchTime - 130);
        } else if (currentMatchTime > 105) {
          return Seconds.of(0);
        } else if (currentMatchTime > 80) {
          return Seconds.of(currentMatchTime - 80);
        } else if (currentMatchTime > 55) {
          return Seconds.of(0);
        }
      }
    }

    if (autonomousWinnerIsRed.isEmpty()) {
      // Assume active to not disable robot performance if auto winner is unknown
      return Seconds.of(999);
    }

    // Already inactive or no FMS
    return Seconds.of(0);
  }

  @SuppressWarnings("unused")
  public static boolean fmsAttached() {
    return kSimulatePracticeMatch || DriverStation.isFMSAttached();
  }

  public static void setAutoWinner(boolean redAlliance) {
    autonomousWinnerIsRed = Optional.of(redAlliance);
  }

  public static void updateAutonomousWinner() {
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          autonomousWinnerIsRed = Optional.of(false);
          break;
        case 'R':
          autonomousWinnerIsRed = Optional.of(true);
          break;
      }
    }
  }
}
