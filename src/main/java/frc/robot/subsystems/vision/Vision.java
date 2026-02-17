package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem responsible for: reading AprilTag detections from PhotonVision; estimating robot
 * field pose; dynamically scaling measurement trust; and feeding vision measurements into the
 * drivetrain pose estimator
 *
 * <p>One instance of this class represents a single physical camera.
 */
@Logged
public class Vision extends SubsystemBase {

  private final VisionIO[] cameras;

  // private final HootAutoReplay autoReplay;

  /**
   * @param cameraName Name of the PhotonVision camera (must match NT name exactly)
   * @param robotToCamera Transform from robot center to camera (meters, radians)
   * @param drivetrain Reference to drivetrain for pose fusion
   */
  public Vision(VisionIO[] cameras) {
    this.cameras = cameras;
  }

  @Override
  /**
   * Runs every scheduler loop. 1. Fetch fresh vision data (unless replaying logs) 2. Update
   * replay/log values 3. Process and inject pose measurements into drivetrain
   */
  public void periodic() {
    for (VisionIO camera : cameras) {
      camera.process();
    }
  }
}
