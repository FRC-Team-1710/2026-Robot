package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.Log;
import java.util.Optional;

public class MatchState {
  // TODO: Set to simulate a match enviornment to test match specific code
  public static boolean simulatePracticeMatch = true;

  public static Optional<Boolean> autonomousWinnerIsRed = Optional.empty();

  public static boolean isActive() {
    return timeTillActive().in(Seconds) == 0;
  }

  public static Time timeTillActive() {
    Log.log("AutoWinnerIsRed", autonomousWinnerIsRed.isPresent() && autonomousWinnerIsRed.get());
    if (fmsAttached()) {
      var currentMatchTime = DriverStation.getMatchTime();
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
    // Already active or no FMS
    return Seconds.of(0);
  }

  public static Time timeTillInactive() {
    Log.log("AutoWinnerIsRed", autonomousWinnerIsRed.isPresent() && autonomousWinnerIsRed.get());
    if (fmsAttached()) {
      var currentMatchTime = DriverStation.getMatchTime();
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
    // Already inactive or no FMS
    return Seconds.of(0);
  }

  public static boolean fmsAttached() {
    return simulatePracticeMatch || DriverStation.isFMSAttached();
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
