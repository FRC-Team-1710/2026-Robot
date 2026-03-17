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
  private static final boolean m_ignoreFMS = true;

  public static Optional<Boolean> autonomousWinnerIsRed = Optional.empty();

  // Mechanical Advantage's estimate
  public static final double kFuelProcessTime = 1.5;

  // Time after hub inactive until it stops counting
  public static final double kHubCountingEndOffset = 2;

  public static final double kFuelTimeOffset = kFuelProcessTime - kHubCountingEndOffset;

  private static final Timer m_teleopTimer = new Timer();

  /**
   * Returns whether the current match state is considered active.
   *
   * @return true if the match is active, false otherwise.
   */
  public static boolean isActive() {
    return timeTillActive().in(Seconds) == 0;
  }

  /** Starts or restarts the teleoperated period timer. */
  public static void startTeleop() {
    m_teleopTimer.restart();
  }

  /**
   * Determines whether the robot is allowed to shoot, taking into account flight time and the
   * current match state.
   *
   * @param tof The time of flight of the shot in seconds.
   * @return true if the robot can shoot, false otherwise.
   */
  public static boolean canShoot(double tof) {
    // Default to true
    if (autonomousWinnerIsRed.isEmpty() || !fmsAttached() || m_ignoreFMS) {
      return true;
    }
    return timeTillInactive(tof).in(Seconds) > 0;
  }

  /**
   * Returns the time until the match becomes active, ignoring time of flight.
   *
   * @return Time until active.
   */
  public static final Time timeTillActive() {
    return timeTillActive(0);
  }

  /**
   * Returns the time until the match becomes inactive, ignoring time of flight.
   *
   * @return Time until inactive.
   */
  public static final Time timeTillInactive() {
    return timeTillInactive(0);
  }

  /**
   * Returns the time until the match becomes active.
   *
   * @param tof The time of flight of the shot in seconds.
   * @return Time until active.
   */
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

  /**
   * Returns the time until the match becomes inactive.
   *
   * <p>When FMS is attached but the autonomous winner is unknown, this method returns a large
   * sentinel value to indicate that the match should be treated as active. When there is no FMS or
   * the match is already inactive, this method returns zero.
   *
   * @param tof The time of flight of the shot in seconds.
   * @return Time until inactive.
   */
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

      // Unknown auto winner but assume active; return large sentinel
      return Seconds.of(999);
    }

    // Already inactive or no FMS
    return Seconds.of(0);
  }

  @SuppressWarnings("unused")
  /**
   * Returns whether the robot is connected to an FMS or is simulating a practice match.
   *
   * @return true if FMS is attached or simulation is enabled, false otherwise.
   */
  public static boolean fmsAttached() {
    return kSimulatePracticeMatch || DriverStation.isFMSAttached();
  }

  /**
   * Sets the autonomous winner for the current match.
   *
   * @param redAlliance true if the red alliance won autonomous, false otherwise.
   */
  public static void setAutoWinner(boolean redAlliance) {
    autonomousWinnerIsRed = Optional.of(redAlliance);
  }

  /** Updates the autonomous winner based on the game-specific message from the FMS. */
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
