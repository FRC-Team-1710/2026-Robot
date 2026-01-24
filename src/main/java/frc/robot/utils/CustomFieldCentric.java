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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Robot;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveStates;

@Logged
public class CustomFieldCentric implements SwerveRequest {
  public Rotation2d rotationTarget = Rotation2d.kZero;

  public Pose2d currentBumpLocation = Pose2d.kZero;

  public LinearVelocity xVelocity = MetersPerSecond.of(0);
  public LinearVelocity yVelocity = MetersPerSecond.of(0);
  public AngularVelocity angularVelocity = RadiansPerSecond.of(0);

  @NotLogged public final LinearVelocity bumpEmptyVelocity = TunerConstants.kSpeedAt12Volts;
  @NotLogged public final LinearVelocity bumpFullVelocity = bumpEmptyVelocity.times(0.75);

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

  @NotLogged private boolean shouldResetYAssistPID = true;
  @NotLogged private boolean shouldResetRotationPID = true;

  private double maxBumpSpeed = 0;

  public RequestStates currentDriveState = RequestStates.DRIVER_CONTROLLED;

  private final SwerveRequest.ApplyFieldSpeeds driveRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.Position);

  public CustomFieldCentric() {
    rotationLockPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  @SuppressWarnings("unused")
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    if (currentDriveState != RequestStates.ROTATION_LOCK
        && CommandSwerveDrivetrain.kAutoBumpAlignment) {
      // if (xVelocity.in(MetersPerSecond) != 0 || yVelocity.in(MetersPerSecond) != 0) {
      if (Math.hypot(
              Math.abs(xVelocity.in(MetersPerSecond)), Math.abs(yVelocity.in(MetersPerSecond)))
          > TunerConstants.kSpeedAt12Volts.times(0.5).in(MetersPerSecond)) {
        Robot.telemetry()
            .log(
                "CustomFieldCentric/RobotEyes",
                new Pose2d(
                        parameters.currentPose.getTranslation(),
                        new Rotation2d(xVelocity.in(MetersPerSecond), yVelocity.in(MetersPerSecond))
                            .plus(Rotation2d.kCCW_90deg))
                    .plus(new Transform2d(0, -2.5, Rotation2d.kZero)),
                Pose2d.struct);
        Robot.telemetry()
            .log(
                "CustomFieldCentric/StillGoingOverBump",
                stillGoingOverBump(parameters.currentPose.getTranslation()));
        boolean towardsBump =
            towardsBump(
                new Pose2d(
                    parameters.currentPose.getTranslation(),
                    new Rotation2d(xVelocity.in(MetersPerSecond), yVelocity.in(MetersPerSecond))
                        .plus(Rotation2d.kCW_90deg)));
        Robot.telemetry().log("CustomFieldCentric/TowardsBump", towardsBump);
        if (towardsBump
            || (currentDriveState == RequestStates.BUMP_ASSIST
                && stillGoingOverBump(parameters.currentPose.getTranslation()))) {
          currentDriveState = RequestStates.BUMP_ASSIST;
        } else {
          currentDriveState = RequestStates.DRIVER_CONTROLLED;
        }
      } else {
        currentDriveState = RequestStates.DRIVER_CONTROLLED;
      }
    }

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
      case BUMP_ASSIST:
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
                yVelocity
                    .times(0.75)
                    .plus(
                        MetersPerSecond.of(
                            yAssistPID.calculate(
                                parameters.currentPose.getY(), currentBumpLocation.getY()))),
                angularVelocity.plus(
                    RadiansPerSecond.of(rotationLockPID.calculate(currentRadians))));
        // Apply max bump speed only to the forward velocity (uncapped strafe velocity)
        maxBumpSpeed = getMaxSpeedForBump(parameters.currentPose).in(MetersPerSecond);
        wantedSpeeds.vxMetersPerSecond =
            Math.abs(xVelocity.in(MetersPerSecond)) > maxBumpSpeed
                ? Math.copySign(maxBumpSpeed, xVelocity.in(MetersPerSecond))
                : xVelocity.in(MetersPerSecond);
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

  private boolean stillGoingOverBump(Translation2d translation) {
    return ((translation.getX()
            > currentBumpLocation.getX() - FieldConstants.kBumpDepth.div(2).in(Meters))
        && (translation.getX()
            < currentBumpLocation.getX() + FieldConstants.kBumpDepth.div(2).in(Meters)));
  }

  private boolean towardsBump(Pose2d robotWantedVelocityHeading) {
    var targetBump =
        FieldConstants.Bump.BumpLocation.getClosest(robotWantedVelocityHeading.getTranslation());
    currentBumpLocation = new Pose2d(targetBump.average, Rotation2d.kZero);
    Robot.telemetry()
        .log("CustomFieldCentric/TowardsBumpLocation", currentBumpLocation, Pose2d.struct);
    Robot.telemetry()
        .log(
            "CustomFieldCentric/RobotWantedVelocityHeading",
            robotWantedVelocityHeading,
            Pose2d.struct);
    return MathUtils.willPenetrateLine(
            robotWantedVelocityHeading, targetBump.translationOutside, targetBump.translationInside)
        && (currentBumpLocation
                .getTranslation()
                .getDistance(robotWantedVelocityHeading.getTranslation())
            < 2.5);
  }

  private LinearVelocity getMaxSpeedForBump(Pose2d currentPose) {
    if ((currentPose.getX() > FieldConstants.kFieldLength.div(2).in(Meters) && Alliance.redAlliance)
        || (currentPose.getX() < FieldConstants.kFieldLength.div(2).in(Meters)
            && !Alliance.redAlliance)) {
      return ((currentPose.getX() < FieldConstants.kBumpDistanceFromDS.in(Meters))
              || (currentPose.getX()
                  > FieldConstants.kFieldLength
                      .minus(FieldConstants.kBumpDistanceFromDS)
                      .in(Meters)))
          ? bumpEmptyVelocity
          : bumpFullVelocity;
    }
    return bumpEmptyVelocity;
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
    switch (state) {
      case DRIVER_CONTROLLED:
        // If it was rotation lock, set back to driver control
        if (this.currentDriveState == RequestStates.ROTATION_LOCK) {
          this.currentDriveState = RequestStates.DRIVER_CONTROLLED;
        }
        // Only reset PID if it's not targeting
        if (this.currentDriveState == RequestStates.DRIVER_CONTROLLED) {
          this.shouldResetRotationPID = true;
          this.shouldResetYAssistPID = true;
        }
        break;
      case ROTATION_LOCK:
        // Reset PID if it wasn't already rotation locked
        if (this.currentDriveState != RequestStates.ROTATION_LOCK) {
          this.shouldResetRotationPID = true;
        }
        this.currentDriveState = RequestStates.ROTATION_LOCK;
        this.shouldResetYAssistPID = true;
        break;
    }
    return this;
  }

  public enum RequestStates {
    DRIVER_CONTROLLED,
    BUMP_ASSIST,
    ROTATION_LOCK
  }
}
