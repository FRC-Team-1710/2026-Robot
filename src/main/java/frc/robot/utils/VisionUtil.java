// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

/** import frc.robot. */

/**
 * Utility class for vision processing that provides different vision modes, measurement validation,
 * and data structures for vision measurements. Supports both MegaTag1 (MT1) and MegaTag2 (MT2)
 * vision systems.
 */
@Logged
public class VisionUtil {
  @Logged(importance = Importance.DEBUG)
  public static final Distance FIELD_MARGIN =
      Meters.of(0.5); // Meters beyond field boundaries to accept measurements

  @Logged(importance = Importance.DEBUG)
  public static final Distance Z_MARGIN =
      Meters.of(0.5); // Meters above/below field to accept measurements

  @Logged(importance = Importance.DEBUG)
  public static final AngularVelocity MT2_SPIN_MAX =
      DegreesPerSecond.of(40.0); // Maximum rotation speed for MT2 measurements

  @Logged(importance = Importance.DEBUG)
  public static final double MIN_TAG_AREA = 0.1; // Minimum tag area to be accepted

  // Vision measurement constants for MA mode
  @Logged(importance = Importance.DEBUG)
  private static final double MA_VISION_STD_DEV_XY = 0.333; // Base XY standard deviation

  @SuppressWarnings("unused")
  @Logged(importance = Importance.DEBUG)
  private static final double MA_VISION_STD_DEV_THETA =
      Double.MAX_VALUE; // Base theta standard deviation

  @Logged(importance = Importance.DEBUG)
  public static final double MA_AMBIGUITY =
      0.3; // Maximum allowed ambiguity for single-tag measurements

  @Logged(importance = Importance.DEBUG)
  public static Transform2d getTagOffset(
      Transform3d cameraToTag, Transform3d visionStdDev, Transform2d desiredOffset) {
    Transform3d robotToTargetPose = cameraToTag.plus(visionStdDev);
    Transform2d robotOffset =
        new Transform2d(
            new Translation2d(
                robotToTargetPose.getX() - desiredOffset.getX(),
                robotToTargetPose.getY() - desiredOffset.getY()),
            new Rotation2d(
                robotToTargetPose.getRotation().getZ() - desiredOffset.getRotation().getRadians()));
    return robotOffset;
  }
}
