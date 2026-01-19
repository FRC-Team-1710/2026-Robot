package frc.robot.autonomous;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class BLineRequest implements SwerveRequest {
  public Pose2d TargetPose = Pose2d.kZero;

  public final SwerveRequest.ApplyFieldSpeeds driveRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  Path testPath = new Path("testPath");

  public Command runAuto(CommandSwerveDrivetrain drivetrain) {

    return Commands.sequence(drivetrain.pathBuilder.build(testPath));
  }

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
