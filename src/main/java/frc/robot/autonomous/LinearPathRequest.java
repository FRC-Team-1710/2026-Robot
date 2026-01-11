package frc.robot.autonomous;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.LinearPath;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LinearPathRequest implements SwerveRequest {

  public Pose2d TargetPose = Pose2d.kZero;

  // PID Controllers
  public PhoenixPIDController XController = new PhoenixPIDController(10, 0, 0);
  public PhoenixPIDController YController = new PhoenixPIDController(10, 0, 0);
  public PhoenixPIDController ThetaController = new PhoenixPIDController(7, 0, 0);

  private final WheelForceCalculator forceCalculator;
  private final LinearPath path;

  // State
  private double elapsedTime = 0;
  private LinearPath.State initialState = new LinearPath.State();
  private ChassisSpeeds lastSetpointSpeeds = new ChassisSpeeds();
  private LinearPath.State setpoint = new LinearPath.State();

  // Underlying request
  private final SwerveRequest.ApplyFieldSpeeds driveRequest =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  public LinearPathRequest(
      TrapezoidProfile.Constraints linearProfile,
      TrapezoidProfile.Constraints angularProfile,
      WheelForceCalculator forceCalculator) {

    this.forceCalculator = forceCalculator;
    this.path = new LinearPath(linearProfile, angularProfile);
    ThetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void reset(Pose2d currentPose, ChassisSpeeds fieldSpeeds) {
    initialState =
        new LinearPath.State(
            currentPose, fieldSpeeds);

    elapsedTime = 0.0;
    // Initialize setpoint to current state
    setpoint = path.calculate(elapsedTime, initialState, TargetPose);
    lastSetpointSpeeds = setpoint.speeds;
  }

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    double dt = parameters.updatePeriod;
    elapsedTime += dt;

    // Update setpoint to get speed robot should be running at based on current time
    setpoint = path.calculate(elapsedTime, initialState, TargetPose);

    Pose2d currentPose = parameters.currentPose;

    // Calculate feedback corrections
    double xFeedback =
        XController.calculate(currentPose.getX(), setpoint.pose.getX(), parameters.timestamp);
    double yFeedback =
        YController.calculate(currentPose.getY(), setpoint.pose.getY(), parameters.timestamp);
    double thetaFeedback =
        ThetaController.calculate(
            currentPose.getRotation().getRadians(),
            setpoint.pose.getRotation().getRadians(),
            parameters.timestamp);

    ChassisSpeeds feedbackSpeeds = new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback);

    // Calculate feedforward forces based on PLANNED trajectory change
    Feedforwards feedforwards = forceCalculator.calculate(dt, lastSetpointSpeeds, setpoint.speeds);

    // Combine feedforward + feedback
    ChassisSpeeds correctedSpeeds =
        new ChassisSpeeds(
            setpoint.speeds.vxMetersPerSecond + feedbackSpeeds.vxMetersPerSecond,
            setpoint.speeds.vyMetersPerSecond + feedbackSpeeds.vyMetersPerSecond,
            setpoint.speeds.omegaRadiansPerSecond + feedbackSpeeds.omegaRadiansPerSecond);

    // Save setpoint speeds for next feedforward calculation
    lastSetpointSpeeds = setpoint.speeds;

    // Apply feedforward forces (from planned trajectory) + corrected speeds
    // (planned + feedback)
    return driveRequest
        .withSpeeds(correctedSpeeds)
        .withWheelForceFeedforwardsX(feedforwards.x_newtons)
        .withWheelForceFeedforwardsY(feedforwards.y_newtons)
        .apply(parameters, modulesToApply);
  }

  public boolean isFinished() {
    return path.isFinished(elapsedTime);
  }

  public LinearPathRequest withTargetPose(Pose2d targetPose) {
    this.TargetPose = targetPose;
    return this;
  }

  public LinearPathRequest withXController(PhoenixPIDController controller) {
    this.XController = controller;
    return this;
  }

  public LinearPathRequest withYController(PhoenixPIDController controller) {
    this.YController = controller;
    return this;
  }

  public LinearPathRequest withThetaController(PhoenixPIDController controller) {
    this.ThetaController = controller;
    return this;
  }
}
