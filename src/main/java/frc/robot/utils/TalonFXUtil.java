package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Utility class for common TalonFX motor operations.
 *
 * <p>Provides helper methods for motor configuration, follower setup, and other common patterns
 * used across subsystems.
 */
public final class TalonFXUtil {

  private TalonFXUtil() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Applies a configuration to a TalonFX motor with automatic retries.
   *
   * @param motor The motor to configure
   * @param config The configuration to apply
   * @param maxRetries Maximum number of retry attempts (default: 5)
   * @return true if configuration was successfully applied, false otherwise
   */
  public static boolean applyConfigWithRetries(
      TalonFX motor, TalonFXConfiguration config, int maxRetries) {
    for (int i = 0; i < maxRetries; i++) {
      StatusCode status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Applies a configuration to a TalonFX motor with default retry count (5).
   *
   * @param motor The motor to configure
   * @param config The configuration to apply
   * @return true if configuration was successfully applied, false otherwise
   */
  public static boolean applyConfigWithRetries(TalonFX motor, TalonFXConfiguration config) {
    return applyConfigWithRetries(motor, config, 5);
  }

  public static BaseStatusSignal[] getBasicStatusSignals(TalonFX... motors) {
    BaseStatusSignal[] signals = new BaseStatusSignal[motors.length * 5];
    for (int i = 0; i < motors.length; i++) {
      signals[i] = motors[i].getVelocity();
      signals[i + motors.length] = motors[i].getPosition();
      signals[i + (2 * motors.length)] = motors[i].getStatorCurrent();
      signals[i + (3 * motors.length)] = motors[i].getSupplyCurrent();
      signals[i + (4 * motors.length)] = motors[i].getMotorVoltage();
    }
    return signals;
  }
}
