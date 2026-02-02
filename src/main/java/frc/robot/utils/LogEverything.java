package frc.robot.utils;

import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import java.nio.ByteBuffer;
import java.util.HashSet;
import java.util.Set;

public class LogEverything {
  private static Set<String> lastNTRemoteIds = new HashSet<>();
  private static ByteBuffer ntIntBuffer = ByteBuffer.allocate(4);

  public static void logEverythingPossible() {
    EpilogueBackend logger = Robot.telemetry();

    logger.log("DriverStation/DSAttached", DriverStation.isDSAttached());
    logger.log("DriverStation/FMSAttached", DriverStation.isFMSAttached());
    logger.log(
        "DriverStation/Location",
        DriverStation.getLocation().isPresent() ? DriverStation.getLocation().getAsInt() : -1);
    logger.log("DriverStation/EventName", DriverStation.getEventName());
    logger.log("DriverStation/MatchNumber", DriverStation.getMatchNumber());
    logger.log("DriverStation/ReplayNumber", DriverStation.getReplayNumber());
    logger.log("RobotController/FPGARevision", RobotController.getFPGARevision());
    logger.log("RobotController/FPGARevision", RobotController.getFPGARevision());
    logger.log("RobotController/SerialNumber", RobotController.getSerialNumber());
    logger.log("RobotController/TeamNumber", RobotController.getTeamNumber());
    logger.log("RobotController/BatteryVoltage", RobotController.getBatteryVoltage());
    logger.log("RoboRioDataJNI/FPGAButton", RoboRioDataJNI.getFPGAButton());
    logger.log("HAL/SystemActive", HAL.getSystemActive());
    logger.log("HAL/BrownedOut", HAL.getBrownedOut());
    logger.log("HAL/CommsDisableCount", HAL.getCommsDisableCount());
    logger.log("HAL/RSLState", HAL.getRSLState());
    logger.log("HAL/SystemTimeValid", HAL.getSystemTimeValid());
    logger.log("PowerJNI/VinVoltage", PowerJNI.getVinVoltage());
    logger.log("PowerJNI/VinCurrent", PowerJNI.getVinCurrent());
    logger.log("PowerJNI/UserVoltage3V3", PowerJNI.getUserVoltage3V3());
    logger.log("PowerJNI/UserCurrent3V3", PowerJNI.getUserCurrent3V3());
    logger.log("PowerJNI/UserActive3V3", PowerJNI.getUserActive3V3());
    logger.log("PowerJNI/UserCurrentFaults3V3", PowerJNI.getUserCurrentFaults3V3());
    logger.log("PowerJNI/UserVoltage5V", PowerJNI.getUserVoltage5V());
    logger.log("PowerJNI/UserCurrent5V", PowerJNI.getUserCurrent5V());
    logger.log("PowerJNI/UserActive5V", PowerJNI.getUserActive5V());
    logger.log("PowerJNI/UserCurrentFaults5V", PowerJNI.getUserCurrentFaults5V());
    logger.log("PowerJNI/UserVoltage6V", PowerJNI.getUserVoltage6V());
    logger.log("PowerJNI/UserCurrent6V", PowerJNI.getUserCurrent6V());
    logger.log("PowerJNI/UserActive6V", PowerJNI.getUserActive6V());
    logger.log("PowerJNI/UserCurrentFaults6V", PowerJNI.getUserCurrentFaults6V());
    logger.log("PowerJNI/BrownoutVoltage", PowerJNI.getBrownoutVoltage());
    logger.log("RobotController/CPUTemp", RobotController.getCPUTemp());
    logger.log(
        "RobotController/CANStatus/PercentBusUtilization",
        RobotController.getCANStatus().percentBusUtilization);
    logger.log("RobotController/CANStatus/BusOffCount", RobotController.getCANStatus().busOffCount);
    logger.log("RobotController/CANStatus/TxFullCount", RobotController.getCANStatus().txFullCount);
    logger.log(
        "RobotController/CANStatus/ReceiveErrorCount",
        RobotController.getCANStatus().receiveErrorCount);
    logger.log(
        "RobotController/CANStatus/TransmitErrorCount",
        RobotController.getCANStatus().transmitErrorCount);

    final String ntClientsTable = "NTConnections/";
    ConnectionInfo[] ntConnections = NetworkTableInstance.getDefault().getConnections();
    Set<String> ntRemoteIds = new HashSet<>();

    for (int i = 0; i < ntConnections.length; i++) {
      lastNTRemoteIds.remove(ntConnections[i].remote_id);
      ntRemoteIds.add(ntConnections[i].remote_id);
      final var ntClientTable = ntClientsTable + ntConnections[i].remote_id;

      logger.log(ntClientTable + "/Connected", true);
      logger.log(ntClientTable + "/IPAddress", ntConnections[i].remote_ip);
      logger.log(ntClientTable + "/RemotePort", ntConnections[i].remote_port);
      ntIntBuffer.rewind();
      logger.log(
          ntClientTable + "/ProtocolVersion",
          ntIntBuffer.putInt(ntConnections[i].protocol_version).array());
    }

    for (var remoteId : lastNTRemoteIds) {
      logger.log(ntClientsTable + remoteId + "/Connected", false);
    }
    lastNTRemoteIds = ntRemoteIds;
  }
}
