package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.VisionUtil.VisionMeasurement;

/**
 * Vision subsystem responsible for: reading AprilTag detections from PhotonVision; estimating robot
 * field pose; dynamically scaling measurement trust; and feeding vision measurements into the
 * drivetrain pose estimator
 *
 * <p>One instance of this class represents a single physical camera.
 */
public class Vision implements Subsystem {
  private final VisionIO[] m_io;

  private final CommandSwerveDrivetrain m_drivetrain;

  public Vision(CommandSwerveDrivetrain drivetrain, VisionIO... io) {
    m_drivetrain = drivetrain;
    m_io = io;
  }

  public void periodic() {
    for (VisionIO io : m_io) {
      io.fetchInputs();
      VisionMeasurement measurement = io.processInputs();
      m_drivetrain.addVisionMeasurement(measurement);
    }
  }
}
