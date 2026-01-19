package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.epilogue.Logged;

/**
 * The hardware abstraction interface for a PhotonVision-based co-processor that provides
 * PhotonPipelineResult objects. There is a one-to-one relationship between each VisionIO object and
 * each co-processor (e.g., Raspberry Pi) running PhotonVision.
 *
 * <p>In the future, this interface may be further abstracted to not be coupled to PhotonVision.
 * Currently, the abstraction is used to simulate vision.
 */
// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.utils.VisionUtil;
import frc.robot.utils.VisionUtil;
import edu.wpi.first.epilogue.Logged;

@Logged
public interface VisionIO {
  public static class VisionIOInputs {
    boolean connected = false;
    PoseEstimate poseEstimateMT1 = new PoseEstimate();
    PoseEstimate poseEstimateMT2 = new PoseEstimate();
    RawFiducial[] rawFiducialsMT1 = new RawFiducial[0];
    RawFiducial[] rawFiducialsMT2 = new RawFiducial[0];
  }

  default void updateInputs(VisionIOInputs inputs) {}
}