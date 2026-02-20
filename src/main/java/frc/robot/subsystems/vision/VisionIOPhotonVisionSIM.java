package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSIM implements VisionIO {
  private final VisionSystemSim m_visionSim;
  private CommandSwerveDrivetrain m_drivetrain;

  public VisionIOPhotonVisionSIM() {
    this.m_visionSim = new VisionSystemSim("main");
  }

  @Override
  public boolean isConnected() {
    return false;
  }

  @Override
  public void process() {
    m_drivetrain.addVisionMeasurement(new Pose2d(), 0);
  }

  @Override
  public void setConsumer(CommandSwerveDrivetrain consumer) {
    this.m_drivetrain = consumer;
  }
}
