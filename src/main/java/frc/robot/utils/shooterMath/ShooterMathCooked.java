package frc.robot.utils.shooterMath;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.SubsystemConstants;

public final class ShooterMathCooked {
  public static Pose2d currentPose = new Pose2d();

  /**
   * Complete solution for a single shooter.
   *
   * @param robotHeading Robot heading to command for the shot.
   * @param hoodAngle Mechanical hood angle to command.
   * @param flywheelOmega Flywheel ω to command.
   */
  public record ShooterSolution(
      Rotation2d robotHeading, Angle hoodAngle, AngularVelocity flywheelOmega, Distance distance) {}

  /** The current solution for the shooter system. */
  public static ShooterSolution currentSolution =
      new ShooterSolution(
          Rotation2d.kCW_90deg,
          SubsystemConstants.ShooterConstants.Hood.Hardware.kHoodMax,
          RotationsPerSecond.of(30),
          Meters.of(0));

  /** The current passing solution for the shooter system. */
  public static ShooterSolution currentPassingSolution = currentSolution;

  /**
   * Compute optimal shooter parameters for a single shooter.
   *
   * @param robotPose Current robot 2-D pose.
   */
  public static void calculate(Pose2d robotPose) {}

  public static void setMorePow(boolean morePow) {}
}
