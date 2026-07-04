package frc.robot.utils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.DrivetrainAccelerationLimits;
import frc.robot.constants.DrivetrainAutomationConstants;
import frc.robot.constants.Mode;
import frc.robot.constants.Mode.CurrentMode;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveStates;

public class CustomFieldCentric implements SwerveRequest {
  public Rotation2d rotationTarget = Rotation2d.kZero;

  public Rotation2d bumpRotationTarget = Rotation2d.kZero;

  public Pose2d currentBumpLocation = Pose2d.kZero;

  public LinearVelocity xVelocity = MetersPerSecond.of(0);

  public LinearVelocity yVelocity = MetersPerSecond.of(0);

  public AngularVelocity angularVelocity = RadiansPerSecond.of(0);

  private final Pigeon2 gyro;

  private boolean m_shouldRaiseIntake = false;

  private boolean m_isGoingTowardsAllianceZone = false;

  private final PIDController yAssistPID =
      new PIDController(
          Mode.currentMode == CurrentMode.SIMULATION ? 15 : 0.0,
          0.0,
          Mode.currentMode == CurrentMode.SIMULATION ? 2 : 0.0);

  private final ProfiledPIDController rotationLockPID =
      new ProfiledPIDController(
          Mode.currentMode == CurrentMode.SIMULATION ? 50 : 9.0,
          0.0,
          Mode.currentMode == CurrentMode.SIMULATION ? 15 : 0.5,
          new Constraints(3, 4));

  private boolean shouldResetYAssistPID = true;
  private boolean shouldResetRotationPID = true;

  private double maxBumpSpeed = 0;

  public RequestStates currentDriveState = RequestStates.DRIVER_CONTROLLED;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  private boolean m_shouldAcceptRobotSpeeds = false;

  private ChassisSpeeds m_robotSpeeds = new ChassisSpeeds();

  /* Logging vars */
  private boolean stillGoingOverBump = false;

  private boolean towardsBump = false;

  private double m_lastLoopTime = 0;

  private ChassisSpeeds wantedSpeeds = new ChassisSpeeds();

  private ChassisSpeeds wantedSpeedsAfterLimits = new ChassisSpeeds();

  private Translation2d previousTargetTranslation = new Translation2d();

  public CustomFieldCentric(Pigeon2 gyro) {
    this.gyro = gyro;
    rotationLockPID.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putData(rotationLockPID);
    SmartDashboard.putNumber("tuning/kBumpSpeed", 3);
  }

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    long loopStartTime = RobotController.getFPGATime();

    if (shouldResetYAssistPID) {
      yAssistPID.reset();
      shouldResetYAssistPID = false;
    }

    if (shouldResetRotationPID) {
      rotationLockPID.reset(
          parameters.currentPose.getRotation().getRadians(),
          parameters.currentChassisSpeed.omegaRadiansPerSecond);
      shouldResetRotationPID = false;
    }

    switch (currentDriveState) {
      case ROTATION_LOCK:
        rotationLockPID.setGoal(rotationTarget.getRadians());

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

    if (DrivetrainAccelerationLimits.shouldLimit()) {
      var wantedTranslationAfterLimits =
          DrivetrainAccelerationLimits.calculateTilt(
              toRobotSpeeds(
                  new Translation2d(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond),
                  parameters.currentPose),
              previousTargetTranslation);
      wantedSpeedsAfterLimits =
          ChassisSpeeds.fromRobotRelativeSpeeds(
              wantedTranslationAfterLimits.getX(),
              wantedTranslationAfterLimits.getY(),
              wantedSpeeds.omegaRadiansPerSecond,
              parameters.currentPose.getRotation());
    } else {
      wantedSpeedsAfterLimits = wantedSpeeds;
    }

    previousTargetTranslation =
        new Translation2d(
            parameters.currentChassisSpeed.vxMetersPerSecond,
            parameters.currentChassisSpeed.vyMetersPerSecond);

    m_lastLoopTime = RobotController.getFPGATime() - loopStartTime;

    // Log CustomFieldCentric state
    Logger.recordOutput("FieldCentric/RotationTarget", rotationTarget.getDegrees());
    Logger.recordOutput("FieldCentric/BumpRotationTarget", bumpRotationTarget.getDegrees());
    Logger.recordOutput("FieldCentric/XVelocity", xVelocity.in(MetersPerSecond));
    Logger.recordOutput("FieldCentric/YVelocity", yVelocity.in(MetersPerSecond));
    Logger.recordOutput("FieldCentric/AngularVelocity", angularVelocity.in(RadiansPerSecond));
    Logger.recordOutput("FieldCentric/CurrentDriveState", currentDriveState.name());
    Logger.recordOutput("FieldCentric/StillGoingOverBump", stillGoingOverBump);
    Logger.recordOutput("FieldCentric/TowardsBump", towardsBump);
    Logger.recordOutput("FieldCentric/LastLoopTime", m_lastLoopTime);
    Logger.recordOutput("FieldCentric/WantedSpeeds", wantedSpeeds, ChassisSpeeds.struct);
    Logger.recordOutput("FieldCentric/WantedSpeedsAfterLimits", wantedSpeedsAfterLimits, ChassisSpeeds.struct);

    return driveRequest
        .withVelocityX(wantedSpeedsAfterLimits.vxMetersPerSecond)
        .withVelocityY(wantedSpeedsAfterLimits.vyMetersPerSecond)
        .withRotationalRate(wantedSpeeds.omegaRadiansPerSecond)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .apply(parameters, modulesToApply);
  }

  private Translation2d toRobotSpeeds(Translation2d translation2d, Pose2d currentPose) {
    return new Translation2d(
        translation2d.getX() * currentPose.getRotation().getCos()
            + translation2d.getY() * currentPose.getRotation().getSin(),
        -translation2d.getX() * currentPose.getRotation().getSin()
            + translation2d.getY() * currentPose.getRotation().getCos());
  }

  public CustomFieldCentric withTargetRotation(Rotation2d target) {
    this.rotationTarget = target;
    return this;
  }

  public CustomFieldCentric withVelocityX(LinearVelocity velocity) {
    this.xVelocity = velocity;
    return this;
  }

  public CustomFieldCentric withVelocityX(double velocity) {
    this.xVelocity = MetersPerSecond.of(velocity);
    return this;
  }

  public CustomFieldCentric withVelocityY(LinearVelocity velocity) {
    this.yVelocity = velocity;
    return this;
  }

  public CustomFieldCentric withVelocityY(double velocity) {
    this.yVelocity = MetersPerSecond.of(velocity);
    return this;
  }

  public CustomFieldCentric withRotationalRate(AngularVelocity velocity) {
    this.angularVelocity = velocity;
    return this;
  }

  public CustomFieldCentric withRotationalRate(double velocity) {
    this.angularVelocity = RadiansPerSecond.of(velocity);
    return this;
  }

  public CustomFieldCentric withDriveState(DriveStates state) {
    switch (state) {
      case DRIVER_CONTROLLED:
        if (this.currentDriveState == RequestStates.ROTATION_LOCK) {
          this.currentDriveState = RequestStates.DRIVER_CONTROLLED;
        }
        if (this.currentDriveState == RequestStates.DRIVER_CONTROLLED) {
          this.shouldResetRotationPID = true;
          this.shouldResetYAssistPID = true;
        }
        break;
      case ROTATION_LOCK:
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
    ROTATION_LOCK,
  }
}