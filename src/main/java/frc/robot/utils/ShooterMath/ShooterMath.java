package frc.robot.utils.ShooterMath;

import frc.robot.constants.ShooterConstants;

/** Utility class for shooter-related mathematical calculations. */
public class ShooterMath {
  // ================== Shooter Math Constants ========================

  /* See Desmos for the shooter math calculations: https://www.desmos.com/calculator/sgkhoo58x4 */
  // Velocity Function Constants (sqrt((kA*x^2)/(kB*x + kC)) + kD)
  private static final double kA = 2.68496;
  private static final double kB = 0.11359;
  private static final double kC = 0.0680981;
  private static final double kD = 0.746731;

  // Angle Function Constants (kE * kF ^ x)
  private static final double kE = 1.60189;
  private static final double kF = 0.889922;

  /** Record representing the shoot state with velocity and angle. */
  public record ShootState(double velocity, double angle) {}

  /** Calculates the shooter velocity based on the horizontal distance to the target. */
  private double findShooterVelocity(double x_distance) {
    return Math.sqrt((kA * x_distance * x_distance) / (kB * x_distance + kC)) + kD;
  }

  /** Calculates the shooter angle based on the horizontal distance to the target. */
  private double findShooterAngle(double x_distance) {
    return kE * Math.pow(kF, x_distance);
  }

  /** Converts velocity in meters per second to RPM based on the shooter wheel diameter. */
  public static double velocityToRPM(double vMetersPerSec) {
    double circumference = Math.PI * ShooterConstants.WHEEL_DIAMETER;
    return (vMetersPerSec / circumference) * 60.0;
  }

  /** Calculates the shoot state (velocity and angle) based on the horizontal distance to the target. */
  public ShootState calculateShootState(double x_distance) {
    double velocity = findShooterVelocity(x_distance);
    double angle = findShooterAngle(x_distance);
    return new ShootState(velocity, angle);
  }
}
