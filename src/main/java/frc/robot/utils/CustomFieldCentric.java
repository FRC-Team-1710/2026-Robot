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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveStates;

@Logged
public class CustomFieldCentric implements SwerveRequest {
  public Distance yTargetFromCenter = Meters.of(0);
  public Rotation2d rotationTarget = Rotation2d.kZero;

  public LinearVelocity xVelocity = MetersPerSecond.of(0);
  public LinearVelocity yVelocity = MetersPerSecond.of(0);
  public AngularVelocity angularVelocity = RadiansPerSecond.of(0);

  private final PIDController yAssistPID =
      new PIDController(
          Mode.currentMode == CurrentMode.SIMULATION ? 15 : 0.0,
          0.0,
          Mode.currentMode == CurrentMode.SIMULATION ? 2 : 0.0);
  private final ProfiledPIDController rotationLockPID =
      new ProfiledPIDController(
          Mode.currentMode == CurrentMode.SIMULATION ? 50 : 0.0,
          0.0,
          Mode.currentMode == CurrentMode.SIMULATION ? 15 : 0.0,
          new Constraints(
              Mode.currentMode == CurrentMode.SIMULATION ? 3 : 0.0,
              Mode.currentMode == CurrentMode.SIMULATION ? 4 : 0.0));

  private boolean shouldResetYAssistPID = true;
  private boolean shouldResetRotationPID = true;

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
    if (shouldResetYAssistPID) {
      yAssistPID.reset();
      shouldResetYAssistPID = false;
    }

    if (shouldResetRotationPID) {
      rotationLockPID.reset(parameters.currentPose.getRotation().getRadians(), 0);
      shouldResetRotationPID = false;
    }

    ChassisSpeeds wantedSpeeds;

    switch (currentDriveState) {
      case Y_ASSIST:
        var halfFieldWidth = FieldConstants.kFieldWidth.div(2).in(Meters);
        double currentRadians = parameters.currentPose.getRotation().getRadians();
        double targetSnapRadians = 0;
        if (currentRadians >= (Math.PI / 4) && currentRadians <= (Math.PI * 3 / 4)) {
          targetSnapRadians = Math.PI / 2;
        } else if (currentRadians <= -(Math.PI / 4) && currentRadians >= -(Math.PI * 3 / 4)) {
          targetSnapRadians = -Math.PI / 2;
        } else if (currentRadians <= -(Math.PI * 3 / 4) || currentRadians >= (Math.PI * 3 / 4)) {
          targetSnapRadians = Math.PI;
        }

        rotationLockPID.setGoal(targetSnapRadians);

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
                angularVelocity
                    .times(0.5)
                    .plus(RadiansPerSecond.of(rotationLockPID.calculate(currentRadians))));
        break;
      case ROTATION_LOCK:
        rotationLockPID.setGoal(rotationTarget.getRadians());

        wantedSpeeds =
            new ChassisSpeeds(
                xVelocity,
                yVelocity,
                angularVelocity.plus(
                    RadiansPerSecond.of(
                        rotationLockPID.calculate(
                            parameters.currentPose.getRotation().getRadians()))));
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
  @NotLogged
  public CustomFieldCentric withYTargetFromCenter(Distance target) {
    this.yTargetFromCenter = target;
    return this;
  }

  /**
   * Sets the Y target from the center of the field
   *
   * @param target The target y distance from the center of the field in meters
   * @return The updated CustomFieldCentric object
   */
  @NotLogged
  public CustomFieldCentric withYTargetFromCenter(double target) {
    this.yTargetFromCenter = Meters.of(target);
    return this;
  }

  /**
   * Sets the target rotation
   *
   * @param target The target rotation
   * @return The updated CustomFieldCentric object
   */
  @NotLogged
  public CustomFieldCentric withTargetRotation(Rotation2d target) {
    this.rotationTarget = target;
    return this;
  }

  /**
   * Sets the X velocity
   *
   * @param velocity The target x velocity
   * @return The updated CustomFieldCentric object
   */
  @NotLogged
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
  @NotLogged
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
  @NotLogged
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
  @NotLogged
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
  @NotLogged
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
  @NotLogged
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
  @NotLogged
  public CustomFieldCentric withDriveState(DriveStates state) {
    if (this.currentDriveState != state) {
      if ((state == DriveStates.Y_ASSIST)) {
        this.shouldResetYAssistPID = true;
        this.shouldResetRotationPID = true;
      } else if (state == DriveStates.ROTATION_LOCK) {
        this.shouldResetRotationPID = true;
      }
    }

    this.currentDriveState = state;
    return this;
  }
}
