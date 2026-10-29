package frc.robot.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;

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
  public static void applyConfigWithRetries(
      TalonFX motor, TalonFXConfiguration config, int maxRetries) {
    for (int i = 0; i < maxRetries; i++) {
      StatusCode status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return;
      }
    }
    DriverStation.reportError(
        "Motor id " + motor.getDeviceID() + " would not accept configs (cooked)", false);
  }

  /**
   * Applies a configuration to a TalonFX motor with default retry count (5).
   *
   * @param motor The motor to configure
   * @param config The configuration to apply
   * @return true if configuration was successfully applied, false otherwise
   */
  public static void applyConfig(TalonFXConfiguration config, TalonFX... motors) {
    for (TalonFX talon : motors) {
      applyConfigWithRetries(talon, config, 5);
    }
  }

  /**
   * Optimizes a TalonFX motor for basic status signals (velocity, position, current, voltage).
   *
   * @param motors The motors to optimize
   */
  public static void optimizeForBasicStatusSignals(TalonFX... motors) {
    BaseStatusSignal[] signals = new BaseStatusSignal[motors.length * 5]; // 5 signals each
    for (int i = 0; i < motors.length; i++) {
      signals[i] = motors[i].getVelocity();
      signals[i + motors.length] = motors[i].getPosition();
      signals[i + (2 * motors.length)] = motors[i].getStatorCurrent();
      signals[i + (3 * motors.length)] = motors[i].getSupplyCurrent();
      signals[i + (4 * motors.length)] = motors[i].getMotorVoltage();
    }
    BaseStatusSignal.setUpdateFrequencyForAll(50, signals);
    for (TalonFX motor : motors) {
      motor.optimizeBusUtilization();
    }
  }

  /**
   * Optimizes a TalonFX motor for PID status signals (velocity, position, current, voltage,
   * closed-loop error, closed-loop reference).
   *
   * @param motors The motors to optimize
   */
  public static void optimizeForPIDStatusSignals(TalonFX... motors) {
    BaseStatusSignal[] signals = new BaseStatusSignal[motors.length * 7]; // 7 signals each
    for (int i = 0; i < motors.length; i++) {
      signals[i] = motors[i].getVelocity();
      signals[i + motors.length] = motors[i].getPosition();
      signals[i + (2 * motors.length)] = motors[i].getStatorCurrent();
      signals[i + (3 * motors.length)] = motors[i].getSupplyCurrent();
      signals[i + (4 * motors.length)] = motors[i].getMotorVoltage();
      signals[i + (5 * motors.length)] = motors[i].getClosedLoopError();
      signals[i + (6 * motors.length)] = motors[i].getClosedLoopReference();
    }
    BaseStatusSignal.setUpdateFrequencyForAll(50, signals);
    for (TalonFX motor : motors) {
      motor.optimizeBusUtilization();
    }
  }

  /**
   * Optimizes a TalonFX motor for PID status signals (velocity, position, current, voltage,
   * closed-loop error, closed-loop reference).
   *
   * @param motors The motors to optimize
   */
  public static void optimizeForPIDStatusSignalsExcludingVoltage(TalonFX... motors) {
    BaseStatusSignal[] signals = new BaseStatusSignal[motors.length * 6]; // 6 signals each
    for (int i = 0; i < motors.length; i++) {
      signals[i] = motors[i].getVelocity();
      signals[i + motors.length] = motors[i].getPosition();
      signals[i + (2 * motors.length)] = motors[i].getStatorCurrent();
      signals[i + (3 * motors.length)] = motors[i].getSupplyCurrent();
      signals[i + (4 * motors.length)] = motors[i].getClosedLoopError();
      signals[i + (5 * motors.length)] = motors[i].getClosedLoopReference();
    }
    BaseStatusSignal.setUpdateFrequencyForAll(50, signals);
    for (TalonFX motor : motors) {
      motor.optimizeBusUtilization();
    }
  }
}
