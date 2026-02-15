package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Alliance;
import frc.robot.constants.DrivetrainAutomationConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveStates;

@Logged
public class CustomFieldCentric implements SwerveRequest {
  public Rotation2d rotationTarget = Rotation2d.kZero;

  public Pose2d currentBumpLocation = Pose2d.kZero;

  public LinearVelocity xVelocity = MetersPerSecond.of(0);
  public LinearVelocity yVelocity = MetersPerSecond.of(0);
  public AngularVelocity angularVelocity = RadiansPerSecond.of(0);

  private final Pigeon2 gyro;

  private final PIDController yAssistPID =
      new PIDController(
          Mode.currentMode == CurrentMode.SIMULATION ? 15 : 0.0,
          0.0,
          Mode.currentMode == CurrentMode.SIMULATION ? 2 : 0.0);

  private final ProfiledPIDController rotationLockPID =
      new ProfiledPIDController(
          Mode.currentMode == CurrentMode.SIMULATION ? 50 : 7,
          0.0,
          Mode.currentMode == CurrentMode.SIMULATION ? 15 : 0.0,
          new Constraints(
              // Mode.currentMode == CurrentMode.SIMULATION ?
              3,
              //  : 0.0,
              // Mode.currentMode == CurrentMode.SIMULATION ?
              4
              //  : 0.0
              ));

  @NotLogged private boolean shouldResetYAssistPID = true;
  @NotLogged private boolean shouldResetRotationPID = true;

  private double maxBumpSpeed = 0;

  public RequestStates currentDriveState = RequestStates.DRIVER_CONTROLLED;

  private final SwerveRequest.ApplyFieldSpeeds driveRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.Position);

  /* Logging vars */
  private boolean stillGoingOverBump = false;
  private boolean towardsBump = false;

  @SuppressWarnings("unused")
  private double m_lastLoopTime = 0;

  @SuppressWarnings("unused")
  private Pose2d m_poseLookingForBump = Pose2d.kZero;

  public CustomFieldCentric(Pigeon2 gyro) {
    this.gyro = gyro;
    // Enable PID wrap from -180 to 180 deg
    rotationLockPID.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData(rotationLockPID);
  }

  @Override
  @SuppressWarnings("unused")
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    long loopStartTime = RobotController.getFPGATime();
    if (currentDriveState != RequestStates.ROTATION_LOCK
        && DrivetrainAutomationConstants.BumpDetection.kAutoBumpAlignment) {
      if (Math.hypot(
                  Math.abs(xVelocity.in(MetersPerSecond)), Math.abs(yVelocity.in(MetersPerSecond)))
              >= DrivetrainAutomationConstants.BumpDetection.kMinimumSpeedRequest.in(
                  MetersPerSecond)
          && (Math.hypot(
                      Math.abs(xVelocity.in(MetersPerSecond)),
                      Math.abs(yVelocity.in(MetersPerSecond)))
                  >= DrivetrainAutomationConstants.BumpDetection.kMinimumSpeed.in(MetersPerSecond)
              || currentDriveState == RequestStates.BUMP_ASSIST)) {
        m_poseLookingForBump =
            new Pose2d(
                    parameters.currentPose.getTranslation(),
                    new Rotation2d(xVelocity.in(MetersPerSecond), yVelocity.in(MetersPerSecond))
                        .plus(Rotation2d.kCCW_90deg))
                .plus(new Transform2d(0, -2.5, Rotation2d.kZero));
        stillGoingOverBump =
            stillGoingOverBump(
                parameters.currentPose.getTranslation(),
                gyro.getPitch().getValue(),
                gyro.getRoll().getValue());
        towardsBump =
            towardsBump(
                new Pose2d(
                    parameters.currentPose.getTranslation(),
                    new Rotation2d(xVelocity.in(MetersPerSecond), yVelocity.in(MetersPerSecond))
                        .plus(Rotation2d.kCW_90deg)));
        if (towardsBump || (currentDriveState == RequestStates.BUMP_ASSIST && stillGoingOverBump)) {
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
        // Snap to closest Pi/2 (90 degrees)
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

        maxBumpSpeed = getMaxSpeedForBump(parameters.currentPose).in(MetersPerSecond);

        wantedSpeeds =
            new ChassisSpeeds(
                xVelocity,
                yVelocity
                    .times(DrivetrainAutomationConstants.kDriverTranslationOverrideMultiplier)
                    .plus(
                        MetersPerSecond.of(
                            MathUtil.clamp(
                                yAssistPID.calculate(
                                    parameters.currentPose.getY(), currentBumpLocation.getY()),
                                -maxBumpSpeed,
                                maxBumpSpeed))),
                angularVelocity
                    .times(DrivetrainAutomationConstants.kDriverRotationOverrideMultiplier)
                    .plus(
                        RadiansPerSecond.of(
                            MathUtil.clamp(
                                rotationLockPID.calculate(currentRadians),
                                -DrivetrainAutomationConstants.kRotationPIDMax.in(RadiansPerSecond),
                                DrivetrainAutomationConstants.kRotationPIDMax.in(
                                    RadiansPerSecond)))));
        break;
      case ROTATION_LOCK:
        rotationLockPID.setGoal(rotationTarget.getRadians());

        SmartDashboard.putNumber("BHUVBUHYV", rotationLockPID.getSetpoint().position);

        wantedSpeeds =
            new ChassisSpeeds(
                xVelocity,
                yVelocity,
                angularVelocity
                    .times(DrivetrainAutomationConstants.kDriverRotationOverrideMultiplier)
                    .plus(
                        RadiansPerSecond.of(
                            rotationLockPID.calculate(
                                parameters.currentPose.getRotation().getRadians()))));
        break;
      default:
        wantedSpeeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        break;
    }

    m_lastLoopTime = RobotController.getFPGATime() - loopStartTime;

    return driveRequest.withSpeeds(wantedSpeeds).apply(parameters, modulesToApply);
  }

  /**
   * Returns true if the robot pose is over the bump OR the pitch or roll are above a certain
   * threshold (threshold is zero in the sim)
   */
  private boolean stillGoingOverBump(Translation2d translation, Angle pitch, Angle roll) {
    return ((translation.getX()
                > currentBumpLocation.getX() - FieldConstants.kBumpDepth.div(2).in(Meters))
            && (translation.getX()
                < currentBumpLocation.getX() + FieldConstants.kBumpDepth.div(2).in(Meters)))
        || (Math.hypot(Math.abs(pitch.in(Radians)), Math.abs(roll.in(Radians)))
            > DrivetrainAutomationConstants.BumpDetection.kMinimumAngleThreshold.in(Radians));
  }

  /**
   * Returns true if the robot pose with the driver's inputs as the heading would break the plane
   * along the bump (if the wanted speeds point towards the bump)
   */
  private boolean towardsBump(Pose2d robotWantedVelocityHeading) {
    var targetBump =
        FieldConstants.Bump.BumpLocation.getClosest(robotWantedVelocityHeading.getTranslation());
    currentBumpLocation = new Pose2d(targetBump.average, Rotation2d.kZero);
    return MathUtils.willPenetrateLine(
            robotWantedVelocityHeading, targetBump.translationOutside, targetBump.translationInside)
        && (currentBumpLocation
                .getTranslation()
                .getDistance(robotWantedVelocityHeading.getTranslation())
            < 2.5);
  }

  /** Uses the pose to determine the max speed on the bump depending on its current position */
  private LinearVelocity getMaxSpeedForBump(Pose2d currentPose) {
    // If on same half as current alliance
    return ((currentPose.getX() > FieldConstants.kFieldLength.div(2).in(Meters)
                && Alliance.redAlliance)
            || (currentPose.getX() < FieldConstants.kFieldLength.div(2).in(Meters)
                && !Alliance.redAlliance))
        // If in alliance zone
        ? (((currentPose.getX() < FieldConstants.kBumpDistanceFromDS.in(Meters))
                || (currentPose.getX()
                    > FieldConstants.kFieldLength
                        .minus(FieldConstants.kBumpDistanceFromDS)
                        .in(Meters)))
            ? DrivetrainAutomationConstants.BumpDetection.kBumpFast
            : DrivetrainAutomationConstants.BumpDetection.kBumpSlow)
        : DrivetrainAutomationConstants.BumpDetection.kBumpFast;
  }

  /** Uses the pose to determine if it's going towards the alliance zone */
  public boolean isGoingToAllianceZone(Pose2d currentPose) {
    return (currentPose.getX() > FieldConstants.kFieldLength.div(3).times(2).in(Meters)
            && Alliance.redAlliance)
        || (currentPose.getX() < FieldConstants.kFieldLength.div(3).in(Meters)
            && !Alliance.redAlliance);
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
   * @param state The new {@link RequestState}
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

  /**
   * Current state of the swerve request
   *
   * <p>You do NOT need to request {@link #BUMP_ASSIST} even if auto detection is off (it might
   * break something). This swerve request will automatically switch between {@link
   * #DRIVER_CONTROLLED} and {@link #BUMP_ASSIST} if {@link
   * DrivetrainAutomationConstants.BumpDetection#kAutoBumpAlignment kAutoBumpAlignment}. The driver
   * will at all times at least some type (even if it isn't much) of override (ex. add driver input
   * onto pid output).
   *
   * <p>{@link #DRIVER_CONTROLLED}: Driver has full control with no protections
   *
   * <p>{@link #BUMP_ASSIST}: Robot will align along its x axis (the field's y), auto snap to the
   * closest 90°, and cap max speed (if applicable)
   *
   * <p>{@link #ROTATION_LOCK}: Driver has full control of translation but also snaps to the given
   * {@link CustomFieldCentric#withTargetRotation(Rotation2d target) withTargetRotation(Rotation2d
   * target)}
   */
  public enum RequestStates {
    DRIVER_CONTROLLED,
    BUMP_ASSIST,
    ROTATION_LOCK
  }
}
