// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class DrivetrainAutomationConstants {
  public static final double kDriverTranslationOverrideMultiplier = 0.75;
  public static final double kDriverRotationOverrideMultiplier = 1;
  public static final LinearVelocity kTranslationPIDMax =
      TunerConstants.kSpeedAt12Volts.times(0.75);
  public static final AngularVelocity kRotationPIDMax = TunerConstants.kMaxAngularRate.times(0.75);

  public static class BumpDetection {
    public static final boolean kAutoBumpAlignment = false;

    public static final LinearVelocity kBumpFast = TunerConstants.kSpeedAt12Volts;
    public static final LinearVelocity kBumpSlow = TunerConstants.kSpeedAt12Volts.times(0.5);

    public static final LinearVelocity kMinimumSpeedRequest =
        TunerConstants.kSpeedAt12Volts.times(0.7);
    public static final LinearVelocity kMinimumSpeed = TunerConstants.kSpeedAt12Volts.times(0.5);

    public static final Angle kMinimumAngleThreshold = Degrees.of(7.5);
  }
}
