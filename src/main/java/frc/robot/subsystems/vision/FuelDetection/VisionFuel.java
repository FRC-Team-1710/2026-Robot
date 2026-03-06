package frc.robot.subsystems.vision.FuelDetection;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionFuel extends SubsystemBase {

  public static final double SPHERE_DIAMETER_M = Units.inchesToMeters(4.91);
  public static final double SPHERE_RADIUS_M = SPHERE_DIAMETER_M / 2.0;
  private static final double MIN_DOWNWARD_RAY_Z = -0.01;

  private final PhotonCamera camera;
  private Transform3d robotToCamera;
  private final Supplier<Pose2d> robotPoseSupplier;

  private final FuelFieldMap fuelFieldMap = new FuelFieldMap();
  private final Field2d field = new Field2d();

  public VisionFuel(
      PhotonCamera camera, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
    this.camera = camera;
    this.robotToCamera = robotToCamera;
    this.robotPoseSupplier = robotPoseSupplier;

    SmartDashboard.putData("Sphere Vision/Field", field);
    SmartDashboard.putNumber("ima wah", 0);
  }

  @Override
  public void periodic() {
    Pose2d robotPose = robotPoseSupplier.get();

    robotToCamera =
        new Transform3d(
            robotToCamera.getX(),
            robotToCamera.getY(),
            robotToCamera.getZ(),
            new Rotation3d(0, Units.degreesToRadians(SmartDashboard.getNumber("ima wah", 0)), 0));

    List<Pose2d> detections = estimateSpherePoses(robotPose, camera, robotToCamera);

    fuelFieldMap.updateTracks(detections);

    publishDashboard(robotPose);
  }

  public static List<Pose2d> estimateSpherePoses(
      Pose2d robotPose, PhotonCamera cam, Transform3d robotToCam) {
    List<Pose2d> results = new ArrayList<>();
    var photonResult = cam.getLatestResult();
    if (!photonResult.hasTargets()) return results;

    Pose3d robotPose3d =
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));
    Pose3d cameraPose3d = robotPose3d.transformBy(robotToCam);

    double camX = cameraPose3d.getX();
    double camY = cameraPose3d.getY();
    double camZ = cameraPose3d.getZ();
    Rotation3d camRot = cameraPose3d.getRotation();

    if (camZ <= SPHERE_RADIUS_M) return results;

    for (PhotonTrackedTarget target : photonResult.getTargets()) {
      // (Your existing yaw/pitch math remains here)
      double yawRad =
          Units.degreesToRadians(
              (178.37 / 4)
                  - ((target.getMinAreaRectCorners().get(0).x
                              + target.getMinAreaRectCorners().get(1).x)
                          / 2)
                      / 960
                      * 178.37
                      / 2);
      double pitchRad =
          -Units.degreesToRadians(
              (177.74 / 4)
                  - ((target.getMinAreaRectCorners().get(1).y
                              + target.getMinAreaRectCorners().get(2).y)
                          / 2)
                      / 720
                      * 177.74
                      / 2);

      double cosPitch = Math.cos(pitchRad);
      Translation3d rayCamera =
          new Translation3d(
              cosPitch * Math.cos(yawRad), cosPitch * Math.sin(yawRad), Math.sin(pitchRad));
      Translation3d rayField = rayCamera.rotateBy(camRot);

      double rayDz = rayField.getZ();
      if (rayDz > MIN_DOWNWARD_RAY_Z) continue;

      double t = (SPHERE_RADIUS_M - camZ) / rayDz;
      if (t <= 0.0) continue;

      double sphereX = camX + t * rayField.getX();
      double sphereY = camY + t * rayField.getY();

      double dx = robotPose.getX() - sphereX;
      double dy = robotPose.getY() - sphereY;
      results.add(new Pose2d(sphereX, sphereY, new Rotation2d(Math.atan2(dy, dx))));
    }
    return results;
  }

  private void publishDashboard(Pose2d robotPose) {
    List<Pose2d> visiblePoses = new ArrayList<>();
    List<Pose2d> rememberedPoses = new ArrayList<>();

    for (FuelPosition t : fuelFieldMap.getTracks()) {
      if (t.isCurrentlyVisible()) visiblePoses.add(t.getPose());
      else rememberedPoses.add(t.getPose());
    }

    field.getObject("Spheres - Visible").setPoses(visiblePoses);
    field.getObject("Spheres - Remembered").setPoses(rememberedPoses);

    // Dashboard per-track logic and "Closest" logic would go here, calling fuelFieldMap.getTracks()
  }

  public void clearAllTracks() {
    fuelFieldMap.clearAllTracks();
  }
}
