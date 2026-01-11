package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;

/**
 * Superstructure - Controls the Arm and Flywheel together.
 *
 * <p>This coordinates:
 *
 * <ul>
 *   <li>Arm - Moves horizontal and vertical to position game pieces
 *   <li>Flywheel - Spins the shooter wheels at the right speed
 * </ul>
 *
 * <p>Instead of controlling the arm and flywheel separately, this gives you simple
 * commands like "score low" or "prepare for shooting" that move both parts together.
 * This makes driving easier and ensures everything moves in sync.
 */
@Logged
public class Superstructure extends SubsystemBase {

  // ==================== Subsystems ====================
  private final Arm arm;
  private final Flywheel flywheel;

  // ==================== Constructor ====================

  public Superstructure(Arm arm, Flywheel flywheel) {
    this.arm = arm;
    this.flywheel = flywheel;
  }

  // ==================== Coordinated Commands ====================

  /**
   * Command to safely stow the robot for transport.
   * Moves arm to vertical position and stops the flywheel.
   */
  public Command stowCommand() {
    return Commands.parallel(
            arm.vertical(),
            flywheel.stopCommand())
        .withName("Stow");
  }

  /**
   * Command to stow and wait until both mechanisms reach target.
   * Waits for arm to reach vertical and flywheel to stop completely.
   */
  public Command stowAndWaitCommand() {
    return Commands.parallel(
            arm.vertical(),
            flywheel.stopCommand())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("StowAndWait");
  }

  /**
   * Command to score in the amp (controlled slow spin).
   * Moves arm to vertical and spins flywheel slowly for controlled scoring.
   */
  public Command ampScoreCommand() {
    return Commands.parallel(
            arm.vertical(),
            flywheel.ampSpeed())
        .withName("AmpScore");
  }

  /**
   * Command to score in amp and wait until ready.
   * Waits for arm to reach vertical and flywheel to reach amp speed.
   */
  public Command ampScoreAndWaitCommand() {
    return Commands.parallel(
            arm.vertical(),
            flywheel.ampSpeed())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("AmpScoreAndWait");
  }

  /**
   * Command to prepare for close speaker shot.
   * Moves arm to scoring angle (30°) and spins flywheel at medium speed (25 RPS).
   */
  public Command speakerCloseCommand() {
    return Commands.parallel(
            arm.scoringPosition(),
            flywheel.spinUp())
        .withName("SpeakerClose");
  }

  /**
   * Command to prepare for close speaker shot and wait until ready.
   * Waits for arm to reach scoring position and flywheel to reach speed.
   */
  public Command speakerCloseAndWaitCommand() {
    return Commands.parallel(
            arm.scoringPosition(),
            flywheel.spinUp())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("SpeakerCloseAndWait");
  }

  /**
   * Command to prepare for far speaker shot.
   * Moves arm to high angle (45°) and spins flywheel fast (35 RPS).
   */
  public Command speakerFarCommand() {
    return Commands.parallel(
            arm.scoringHighPosition(),
            flywheel.farSpeed())
        .withName("SpeakerFar");
  }

  /**
   * Command to prepare for far speaker shot and wait until ready.
   * Waits for arm to reach high position and flywheel to reach fast speed.
   */
  public Command speakerFarAndWaitCommand() {
    return Commands.parallel(
            arm.scoringHighPosition(),
            flywheel.farSpeed())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("SpeakerFarAndWait");
  }

  /**
   * Command to prepare for ground intake.
   * Moves arm to horizontal position and stops flywheel.
   */
  public Command intakeGroundCommand() {
    return Commands.parallel(
            arm.horizontal(),
            flywheel.stopCommand())
        .withName("IntakeGround");
  }

  /**
   * Command to prepare for ground intake and wait until ready.
   * Waits for arm to reach horizontal and flywheel to stop.
   */
  public Command intakeGroundAndWaitCommand() {
    return Commands.parallel(
            arm.horizontal(),
            flywheel.stopCommand())
        .andThen(Commands.waitUntil(() -> arm.isAtTarget() && flywheel.isAtTarget()))
        .withName("IntakeGroundAndWait");
  }

  // ==================== Action Commands ====================

  /**
   * Command to shoot a game piece.
   * Waits briefly for the game piece to be expelled from the robot.
   *
   * <p>This represents the time needed for the spinning flywheel to launch the game piece.
   * Use this after preparing the shooter with speaker commands.
   */
  public Command shootCommand() {
    return Commands.waitSeconds(0.3).withName("Shoot");
  }
}
