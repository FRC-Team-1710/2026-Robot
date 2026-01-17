package frc.robot.autonomous;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;

public class BLineRequest implements SwerveRequest {
  public Pose2d TargetPose = Pose2d.kZero;

  private final SwerveRequest.ApplyFieldSpeeds driveRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public BLineRequest() {}

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {

    // TODO: Implement BLine path following logic here

    return driveRequest.apply(parameters, modulesToApply);
  }

  public boolean isFinished() {
    // TODO: Implement ending logic here
    return false;
  }

  public BLineRequest withTargetPose(Pose2d targetPose) {
    this.TargetPose = targetPose;
    return this;
  }
}
