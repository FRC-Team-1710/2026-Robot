// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

public class DrivetrainAutomationConstants {
  public static final double kDriverTranslationOverrideMultiplier = 0.25;
  public static final double kDriverRotationOverrideMultiplier = 1;
  public static final AngularVelocity kRotationPIDMax = TunerConstants.kMaxAngularRate.times(0.75);

  public static class BumpDetection {
    private static boolean m_autoBumpAlignment = true;

    public static void toggleAutoBumpAlignment() {
      m_autoBumpAlignment = !m_autoBumpAlignment;
    }

    public static boolean shouldAlignBump() {
      Robot.telemetry().log("shouldAlignBump", m_autoBumpAlignment);
      return m_autoBumpAlignment;
    }

    public static LinearVelocity kBumpSpeed = MetersPerSecond.of(3);

    public static final LinearVelocity kMinimumSpeedRequest =
        TunerConstants.kSpeedAt12Volts.times(0.15);

    public static final Angle kMinimumAngleThreshold = Degrees.of(7.5);

    public static final Rotation2d kSnap1 = Rotation2d.fromDegrees(0);
    public static final Rotation2d kSnap2 = Rotation2d.fromDegrees(180);

    /**
     * Must take less than this long to go to the bump for it to activate. Ex. 0.5 means that from 2
     * meters away the minimum speed required is 4 m/s. This ensures that the robot has enough time
     * to align properly before it reaches the bump.
     */
    public static final double kBumpDetectionTime = 0.75;
  }
}
