package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Waypoints;
import frc.robot.subsystems.Superstructure;

public class AutoRoutines {

  private final AutoCommands autoCommands;
   private final Superstructure superstructure;

  public AutoRoutines(AutoCommands autoCommands, Superstructure superstructure) {
    this.autoCommands = autoCommands;
    this.superstructure = superstructure;
  }

  /**
   * Example autonomous using WAITING commands for sequential operations.
   *
   * <p>Demonstrates using AndWait variants to ensure mechanisms are ready before continuing.
   * Good for when you need precise timing and guaranteed completion.
   */
  public Command sequentialScoringAuto() {
    return Commands.sequence(
        Commands.print("=== Sequential Scoring Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        // Wait for speaker shot to be ready, then shoot
        superstructure.speakerCloseAndWaitCommand(),
        superstructure.shootCommand(),
        // Drive to game piece while preparing intake
        autoCommands.driveTo(new Pose2d(3.0, 0, Rotation2d.kZero)),
        superstructure.intakeGroundAndWaitCommand(),
        Commands.waitSeconds(0.5), // Time to intake game piece
        // Drive back and prepare for far shot
        autoCommands.driveTo(new Pose2d(4.0, 2.0, Rotation2d.kZero)),
        superstructure.speakerFarAndWaitCommand(),
        superstructure.shootCommand(),
        superstructure.stowAndWaitCommand(),
        Commands.print("=== Complete ==="));
  }

  /**
   * Example autonomous using INSTANT commands for parallel preparation.
   *
   * <p>Demonstrates starting mechanism movements while driving to save time.
   * Good for when you want responsive, overlapping actions.
   */
  public Command parallelPreparationAuto() {
    return Commands.sequence(
        Commands.print("=== Parallel Preparation Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        // Start preparing speaker shot immediately (instant command)
        superstructure.speakerCloseCommand(),
        Commands.waitSeconds(1.0), // Give time for mechanisms to get ready
        superstructure.shootCommand(),
        // Drive to game piece and start preparing intake partway through
        Commands.parallel(
            autoCommands.driveTo(new Pose2d(3.0, 0, Rotation2d.kZero)),
            Commands.sequence(
                Commands.waitSeconds(0.5), // Wait partway through drive
                superstructure.intakeGroundCommand() // Start moving intake early
                )),
        Commands.waitSeconds(0.5), // Complete intake
        // Start preparing amp score while driving
        Commands.parallel(
            autoCommands.driveTo(new Pose2d(1.0, 4.0, Rotation2d.kZero)),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                superstructure.ampScoreCommand() // Prepare during drive
                )),
        superstructure.shootCommand(),
        superstructure.stowCommand(),
        Commands.print("=== Complete ==="));
  }

  /**
   * Simple mobility autonomous - just leave the starting zone.
   *
   * <p>Demonstrates basic driving and stowing for safe transport.
   */
  public Command mobilityAuto() {
    return Commands.sequence(
        Commands.print("=== Mobility Auto ==="),
        superstructure.stowCommand(), // Ensure robot is in safe position
        autoCommands.driveTo(Waypoints.MIDFIELD_CENTER),
        Commands.print("=== Complete ==="));
  }

  /**
   * Drive back and forth autonomous - continuously drives between two positions.
   *
   * <p>Demonstrates using .repeatedly() to repeat a sequence indefinitely.
   * Good for testing drivetrain reliability and path following.
   */
  public Command driveBackAndForthAuto() {
    return Commands.sequence(
        Commands.print("=== Drive Back and Forth Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        superstructure.stowCommand(), // Ensure robot is in safe position
        // Drive back and forth repeatedly until autonomous ends
        Commands.sequence(
                Commands.print("Driving forward..."),
                autoCommands.driveTo(Waypoints.MIDFIELD_CENTER),
                Commands.print("Driving back..."),
                autoCommands.driveTo(Pose2d.kZero))
            .repeatedly());
  }

  /**
   * Example showing all 5 command types in sequence.
   *
   * <p>Educational example demonstrating each command's purpose.
   */
  public Command demonstrationAuto() {
    return Commands.sequence(
        Commands.print("=== Demonstration Auto ==="),
        autoCommands.resetPose(Pose2d.kZero),
        // 1. Stow - safe transport
        Commands.print("1. Stowing for transport"),
        superstructure.stowAndWaitCommand(),
        Commands.waitSeconds(0.5),
        // 2. Amp score - slow controlled scoring
        Commands.print("2. Amp scoring (slow speed)"),
        superstructure.ampScoreAndWaitCommand(),
        Commands.waitSeconds(0.5),
        // 3. Speaker close - medium speed shot
        Commands.print("3. Close speaker shot (medium speed)"),
        superstructure.speakerCloseAndWaitCommand(),
        Commands.waitSeconds(0.5),
        // 4. Speaker far - fast long-range shot
        Commands.print("4. Far speaker shot (fast speed)"),
        superstructure.speakerFarAndWaitCommand(),
        Commands.waitSeconds(0.5),
        // 5. Ground intake - horizontal position
        Commands.print("5. Ground intake position"),
        superstructure.intakeGroundAndWaitCommand(),
        Commands.waitSeconds(0.5),
        // Return to stow
        Commands.print("Returning to stow"),
        superstructure.stowAndWaitCommand(),
        Commands.print("=== Complete ==="));
  }
}