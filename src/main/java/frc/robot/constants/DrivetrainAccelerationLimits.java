package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainAccelerationLimits {
  // Default to bump detection as they're both toggled by the same button
  private static boolean shouldLimit =
      DrivetrainAutomationConstants.BumpDetection.shouldAlignBump();

  // Tuning, increase until tipping
  /** List of maximum acceleration limits for each direction (0 = no limit) */
  public static LinearAcceleration[] kMaxAccelerationLimits =
      new LinearAcceleration[] {
        MetersPerSecondPerSecond.of(0), // Forward (shooter)
        MetersPerSecondPerSecond.of(0), // Backward (intake)
        MetersPerSecondPerSecond.of(0), // Left
        MetersPerSecondPerSecond.of(0) // Right
      };

  /** Seconds (250Hz or 4 ms) */
  private static final double DT = 1.0 / 250.0;

  /**
   * Calculates the clamped acceleration for each direction independently, allowing for different
   * limits in each direction and still keeping the same direction and ratio of vx to vy
   *
   * @param target The target robot speeds to achieve
   * @param previousTarget The previous loop's target robot speeds (used to calculate desired
   *     acceleration)
   * @param currentSpeeds The current robot speeds (used to prevent sudden changes in direction)
   * @return The new robot speeds after applying the acceleration limits while still keeping the
   *     same direction and ratio of vx to vy
   */
  public static Translation2d calculateTilt(Translation2d target, Translation2d previousTarget) {
    kMaxAccelerationLimits[0] =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("tuning/acceleration/AccelFront", 0));
    kMaxAccelerationLimits[1] =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("tuning/acceleration/AccelBack", 0));
    kMaxAccelerationLimits[2] =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("tuning/acceleration/AccelLeft", 0));
    kMaxAccelerationLimits[3] =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("tuning/acceleration/AccelRight", 0));

    double dvx = target.getX() - previousTarget.getX();
    double dvy = target.getY() - previousTarget.getY();

    double maxDvx =
        (dvx >= 0 ? kMaxAccelerationLimits[0] : kMaxAccelerationLimits[1])
                .in(MetersPerSecondPerSecond)
            * DT;
    double maxDvy =
        (dvy >= 0 ? kMaxAccelerationLimits[2] : kMaxAccelerationLimits[3])
                .in(MetersPerSecondPerSecond)
            * DT;

    double scale = 1.0;
    if (Math.abs(dvx) > maxDvx) scale = Math.min(scale, maxDvx / Math.abs(dvx));
    if (Math.abs(dvy) > maxDvy) scale = Math.min(scale, maxDvy / Math.abs(dvy));

    return new Translation2d(
        previousTarget.getX() + dvx * scale, previousTarget.getY() + dvy * scale);
  }

  static {
    SmartDashboard.putNumber("tuning/acceleration/AccelFront", 10000);
    SmartDashboard.putNumber("tuning/acceleration/AccelBack", 10000);
    SmartDashboard.putNumber("tuning/acceleration/AccelLeft", 10000);
    SmartDashboard.putNumber("tuning/acceleration/AccelRight", 10000);
  }

  public static boolean shouldLimit() {
    return shouldLimit;
  }

  public static void toggleShouldLimit() {
    shouldLimit = !shouldLimit;
  }
}
