package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class AccelerationLimits {
  // Tuning, start at 10 and increase until robot can't follow target speed
  /** Accounts for motor torque at any given speed */
  public static LinearAcceleration kMaxForwardAcceleration = MetersPerSecondPerSecond.of(0);

  // Tuning, increase until tipping
  /** List of maximum acceleration limits for each direction (0 = no limit) */
  public static LinearAcceleration[] kMaxAccelerationLimits =
      new LinearAcceleration[] {
        MetersPerSecondPerSecond.of(0), // Forward (shooter)
        MetersPerSecondPerSecond.of(0), // Backward (intake)
        MetersPerSecondPerSecond.of(0), // Left
        MetersPerSecondPerSecond.of(0) // Right
      };

  // Tuning, increase until it starts to skid with 90 degree velocity changes (probably won't be
  // used because of new wheels)
  /** Prevent sudden changes in direction */
  public static LinearAcceleration kMaxSkidAcceleration = MetersPerSecondPerSecond.of(0);

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
  public static ChassisSpeeds calculateTilt(ChassisSpeeds target, ChassisSpeeds previousTarget) {
    kMaxAccelerationLimits[0] =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("AccelFront", 0));
    kMaxAccelerationLimits[1] =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("AccelBack", 0));
    kMaxAccelerationLimits[2] =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("AccelLeft", 0));
    kMaxAccelerationLimits[3] =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("AccelRight", 0));

    double dvx = target.vxMetersPerSecond - previousTarget.vxMetersPerSecond;
    double dvy = target.vyMetersPerSecond - previousTarget.vyMetersPerSecond;

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

    return new ChassisSpeeds(
        previousTarget.vxMetersPerSecond + dvx * scale,
        previousTarget.vyMetersPerSecond + dvy * scale,
        target.omegaRadiansPerSecond);
  }

  /**
   * Calculates the clamped acceleration for all motion while keeping the same direction and ratio
   * of vx to vy, allowing for a maximum acceleration in any direction
   *
   * @param target The target robot speeds to achieve
   * @param current The previous loop's target robot speeds (used to calculate desired acceleration)
   * @param currentSpeeds The current robot speeds (used to prevent sudden changes in direction)
   * @return The new robot speeds after applying the acceleration limits while still keeping the
   *     same direction and ratio of vx to vy
   */
  public static ChassisSpeeds calculateForward(
      ChassisSpeeds target, ChassisSpeeds current, ChassisSpeeds currentSpeeds) {
    kMaxForwardAcceleration =
        MetersPerSecondPerSecond.of(SmartDashboard.getNumber("AccelForward", 0));

    double dvx = target.vxMetersPerSecond - current.vxMetersPerSecond;
    double dvy = target.vyMetersPerSecond - current.vyMetersPerSecond;

    double maxDv =
        kMaxForwardAcceleration.in(MetersPerSecondPerSecond)
            * (1.0
                - (Math.hypot(
                        Math.abs(currentSpeeds.vxMetersPerSecond),
                        Math.abs(currentSpeeds.vyMetersPerSecond))
                    / TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)))
            * DT;

    double dvMag = Math.hypot(dvx, dvy);

    double scale = dvMag > maxDv ? maxDv / dvMag : 1.0;

    return new ChassisSpeeds(
        current.vxMetersPerSecond + dvx * scale,
        current.vyMetersPerSecond + dvy * scale,
        target.omegaRadiansPerSecond);
  }
}
