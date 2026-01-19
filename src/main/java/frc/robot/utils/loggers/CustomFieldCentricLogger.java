package frc.robot.utils.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.CustomFieldCentric;

@CustomLoggerFor(CustomFieldCentric.class)
public class CustomFieldCentricLogger extends ClassSpecificLogger<CustomFieldCentric> {
  public CustomFieldCentricLogger() {
    super(CustomFieldCentric.class);
  }

  @Override
  public void update(EpilogueBackend backend, CustomFieldCentric request) {
    backend.log("AngularVelocity", request.angularVelocity);
    backend.log("XVelocity", request.xVelocity);
    backend.log("YVelocity", request.yVelocity);
    backend.log("YTargetFromCenter", request.yTargetFromCenter);
    backend.log("RotationTarget", request.rotationTarget, Rotation2d.struct);
    backend.log("CurrentMode", request.currentDriveState);
  }
}
