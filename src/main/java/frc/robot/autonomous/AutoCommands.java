package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Pre-built driving commands for autonomous mode.
 *
 * <p>Ready-to-use commands for common tasks:
 *
 * <ul>
 *   <li>Drive to a location (driveTo)
 *   <li>Drive while doing something else (driveToWithAction)
 *   <li>Drive then do something when you arrive (driveToThenExecute)
 *   <li>Start actions when close to target (distanceCommand)
 *   <li>Reset the robot's starting position (resetPose)
 * </ul>
 *
 * <p><b>Example:</b>
 * <pre>{@code
 * // Drive to a spot while running the intake
 * driveToWithAction(targetPose, intake.intakeCommand())
 *
 * // Start spinning flywheel when 1.5 meters from shooting position
 * distanceCommand(1.5, shootPose, spinFlywheelCommand())
 * }</pre>
 */
public class AutoCommands {

  // Subsystems
  private final CommandSwerveDrivetrain drivetrain;

  private final LinearPathRequest pathRequest;

  /**
   * Creates AutoCommands using your robot's drive system.
   *
   * @param drivetrain The robot's swerve drive
   * @param pathRequest Helper that plans smooth paths between positions
   */
  public AutoCommands(CommandSwerveDrivetrain drivetrain, LinearPathRequest pathRequest) {
    this.drivetrain = drivetrain;
    this.pathRequest = pathRequest;
  }

  // ==================== Drive Commands ====================

  /**
   * Command to drive to a specific location and angle.
   *
   * @param pose Where to drive (x, y coordinates and rotation)
   * @return Command that drives the robot there
   */
  public Command driveTo(Pose2d pose) {
    return drivetrain.runOnce(() -> pathRequest.reset(drivetrain.getPose(), drivetrain.getFieldSpeeds())).andThen(drivetrain.applyRequest(() -> pathRequest.withTargetPose(pose)));
  }
  
  /**
   * Reset the robot's starting position.
   *
   * @param pose Where the robot should think it is (x, y, rotation)
   * @return Command that resets the position
   */
  public Command resetPose(Pose2d pose) {
    return drivetrain.runOnce(() -> drivetrain.resetPose(pose));
  }

  // ==================== Drive with Parallel Actions ====================

  /**
   * Drive to a location while doing something else at the same time.
   *
   * <p>Both actions start together. Use this when you want to save time by preparing
   * while driving (like raising the arm, spinning up the shooter, or intaking).
   *
   * @param targetPose Where to drive (x, y, and rotation)
   * @param parallelCommand The other thing to do while driving
   * @return Command that does both things at once
   */
  public Command driveToWithAction(Pose2d targetPose, Command parallelCommand) {
    return Commands.parallel(driveTo(targetPose), parallelCommand);
  }

  /**
   * Drive to a location, then do something when you get there.
   *
   * <p>The second action only starts after the robot arrives. Use this for actions that
   * need the robot to be still (like scoring, precise alignment, or shooting).
   *
   * @param targetPose Where to drive (x, y, and rotation)
   * @param afterCommand What to do after arriving
   * @return Command that drives then executes the action
   */
  public Command driveToThenExecute(Pose2d targetPose, Command afterCommand) {
    return Commands.sequence(driveTo(targetPose), afterCommand);
  }

  /**
   * Start a command when the robot gets close to a target location.
   *
   * <p>Use this to prepare ahead of time. For example, start spinning up the shooter
   * before you reach the shooting spot. The command watches the distance and starts
   * the action when you're close enough.
   *
   * <p><b>Example:</b>
   * <pre>{@code
   * // Start spinning flywheel when 1.5 meters away from shooting position
   * Commands.parallel(
   *   driveTo(shootPose),
   *   distanceCommand(1.5, shootPose, spinFlywheelCommand())
   * )
   * }</pre>
   *
   * @param triggerDistance How close to get before starting (in meters)
   * @param targetPose The target location to measure distance from
   * @param command What to do when you get close
   * @return Command that executes when you reach the distance
   */
  public Command distanceCommand(double triggerDistance, Pose2d targetPose, Command command) {
    return Commands.waitUntil(() -> {
      Pose2d currentPose = drivetrain.getPose();
      double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
      return distance <= triggerDistance;
    }).andThen(command);
  }
}
