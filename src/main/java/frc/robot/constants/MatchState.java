package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class MatchState {
  private static final boolean kSimulatePracticeMatch = true;

  public static Optional<Boolean> autonomousWinnerIsRed = Optional.empty();

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
   * Returns the time until the match becomes active, ignoring time of flight.
   *
   * @return Time until active.
   */
  public static final Time timeTillActive() {
    if (fmsAttached()) {
      var currentMatchTime = 140 - m_teleopTimer.get();
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
   * @return Time until inactive.
   */
  public static Time timeTillInactive() {
    if (fmsAttached()) {
      var currentMatchTime = 140 - m_teleopTimer.get();
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
