package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import java.util.Optional;

public class Alliance {
  public static boolean redAlliance = false;
  private static boolean m_hasWarned = false;

  public static void updateRedAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && redAlliance != (alliance.get() == DriverStation.Alliance.Red)) {
      redAlliance = alliance.get() == DriverStation.Alliance.Red;
      Robot.telemetry().log("Alliance/RedAlliance", redAlliance);
    } else if (alliance.isEmpty() && !m_hasWarned) {
      DriverStation.reportWarning("ERROR: Alliance not found. Defaulting to Blue", false);
      redAlliance = false;
      Robot.telemetry().log("Alliance/RedAlliance", redAlliance);
      m_hasWarned = true;
    }
  }
}
