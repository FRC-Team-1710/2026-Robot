package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveStates;

public class CustomFieldCentric implements SwerveRequest {
  public Distance yTargetFromCenter = Meters.of(0);
  public Rotation2d rotationTarget = Rotation2d.kZero;

  public LinearVelocity xVelocity = MetersPerSecond.of(0);
  public LinearVelocity yVelocity = MetersPerSecond.of(0);
  public AngularVelocity angularVelocity = RadiansPerSecond.of(0);

  private final PIDController yAssistPID = new PIDController(0.0, 0.0, 0.0);
  private final ProfiledPIDController rotationLockPID =
      new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0, 0));

  private boolean shouldResetPID = true;

  public DriveStates currentDriveState = DriveStates.DRIVER_CONTROLLED;

  private final SwerveRequest.ApplyFieldSpeeds driveRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.Position);

  public CustomFieldCentric() {
    rotationLockPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    if (shouldResetPID) {
      yAssistPID.reset();
      rotationLockPID.reset(
          parameters.currentPose.getRotation().getRadians(),
          parameters.currentChassisSpeed.omegaRadiansPerSecond);
      shouldResetPID = false;
    }

    ChassisSpeeds wantedSpeeds;

    switch (currentDriveState) {
      case Y_ASSIST:
        var halfFieldWidth = FieldConstants.kFieldWidth.div(2).in(Meters);
        wantedSpeeds =
            new ChassisSpeeds(
                xVelocity,
                yVelocity.plus(
                    MetersPerSecond.of(
                        yAssistPID.calculate(
                            parameters.currentPose.getY(),
                            halfFieldWidth
                                + (parameters.currentPose.getY() > halfFieldWidth
                                        ? yTargetFromCenter
                                        : yTargetFromCenter.times(-1))
                                    .in(Meters)))),
                angularVelocity);
        break;
      case ROTATION_LOCK:
        wantedSpeeds =
            new ChassisSpeeds(
                xVelocity,
                yVelocity,
                angularVelocity.plus(
                    RadiansPerSecond.of(
                        rotationLockPID.calculate(
                            parameters.currentPose.getRotation().getRadians(),
                            rotationTarget.getRadians()))));
        break;
      default:
        wantedSpeeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        break;
    }

    return driveRequest.withSpeeds(wantedSpeeds).apply(parameters, modulesToApply);
  }

  /**
   * Sets the Y target from the center of the field
   *
   * @param target The target y distance from the center of the field
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withYTargetFromCenter(Distance target) {
    if (!this.yTargetFromCenter.equals(target)) {
      this.shouldResetPID = true;
    }
    this.yTargetFromCenter = target;
    return this;
  }

  /**
   * Sets the Y target from the center of the field
   *
   * @param target The target y distance from the center of the field in meters
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withYTargetFromCenter(double target) {
    if (!this.yTargetFromCenter.equals(Meters.of(target))) {
      this.shouldResetPID = true;
    }
    this.yTargetFromCenter = Meters.of(target);
    return this;
  }

  /**
   * Sets the target rotation
   *
   * @param target The target rotation
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withTargetRotation(Rotation2d target) {
    if (!this.rotationTarget.equals(target)) {
      this.shouldResetPID = true;
    }
    this.rotationTarget = target;
    return this;
  }

  /**
   * Sets the X velocity
   *
   * @param velocity The target x velocity
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withVelocityX(LinearVelocity velocity) {
    this.xVelocity = velocity;
    return this;
  }

  /**
   * Sets the X velocity
   *
   * @param velocity The target x velocity in meters/second
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withVelocityX(double velocity) {
    this.xVelocity = MetersPerSecond.of(velocity);
    return this;
  }

  /**
   * Sets the Y velocity
   *
   * @param velocity The target y velocity
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withVelocityY(LinearVelocity velocity) {
    this.yVelocity = velocity;
    return this;
  }

  /**
   * Sets the Y velocity
   *
   * @param velocity The target y velocity in meters/second
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withVelocityY(double velocity) {
    this.yVelocity = MetersPerSecond.of(velocity);
    return this;
  }

  /**
   * Sets the theta velocity
   *
   * @param velocity The target theta velocity
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withRotationalRate(AngularVelocity velocity) {
    this.angularVelocity = velocity;
    return this;
  }

  /**
   * Sets the theta velocity
   *
   * @param velocity The target theta velocity in radians/second
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withRotationalRate(double velocity) {
    this.angularVelocity = RadiansPerSecond.of(velocity);
    return this;
  }

  /**
   * Sets the current drive state
   *
   * @param state The current drive state
   * @return The updated CustomFieldCentric object
   */
  public CustomFieldCentric withDriveState(DriveStates state) {
    if (this.currentDriveState != state
        && (state == DriveStates.Y_ASSIST || state == DriveStates.ROTATION_LOCK)) {
      this.shouldResetPID = true;
    }
    this.currentDriveState = state;
    return this;
  }
}
