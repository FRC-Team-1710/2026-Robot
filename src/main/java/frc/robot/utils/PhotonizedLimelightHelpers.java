package frc.robot.utils;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Barcode;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Classifier;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import java.util.Arrays;

public class PhotonizedLimelightHelpers {

  public static class LimelightResults {

    public String error;

    @JsonProperty("pID")
    public double pipelineID;

    @JsonProperty("tl")
    public double latency_pipeline;

    @JsonProperty("cl")
    public double latency_capture;

    public double latency_jsonParse;

    @JsonProperty("ts")
    public double timestamp_LIMELIGHT_publish;

    @JsonProperty("ts_rio")
    public double timestamp_RIOFPGA_capture;

    @JsonProperty("v")
    @JsonFormat(shape = Shape.NUMBER)
    public boolean valid;

    @JsonProperty("botpose")
    public double[] botpose;

    @JsonProperty("botpose_wpired")
    public double[] botpose_wpired;

    @JsonProperty("botpose_wpiblue")
    public double[] botpose_wpiblue;

    @JsonProperty("botpose_tagcount")
    public double botpose_tagcount;

    @JsonProperty("botpose_span")
    public double botpose_span;

    @JsonProperty("botpose_avgdist")
    public double botpose_avgdist;

    @JsonProperty("botpose_avgarea")
    public double botpose_avgarea;

    @JsonProperty("t6c_rs")
    public double[] camerapose_robotspace;

    public Pose3d getBotPose3d() {
      return toPose3D(botpose);
    }

    public Pose3d getBotPose3d_wpiRed() {
      return toPose3D(botpose_wpired);
    }

    public Pose3d getBotPose3d_wpiBlue() {
      return toPose3D(botpose_wpiblue);
    }

    public Pose2d getBotPose2d() {
      return toPose2D(botpose);
    }

    public Pose2d getBotPose2d_wpiRed() {
      return toPose2D(botpose_wpired);
    }

    public Pose2d getBotPose2d_wpiBlue() {
      return toPose2D(botpose_wpiblue);
    }

    @JsonProperty("Retro")
    public LimelightTarget_Retro[] targets_Retro;

    @JsonProperty("Fiducial")
    public LimelightTarget_Fiducial[] targets_Fiducials;

    @JsonProperty("Classifier")
    public LimelightTarget_Classifier[] targets_Classifier;

    @JsonProperty("Detector")
    public LimelightTarget_Detector[] targets_Detector;

    @JsonProperty("Barcode")
    public LimelightTarget_Barcode[] targets_Barcode;

    // TODO: Get rid of dependancy on limelight
    public LimelightResults() {
      botpose = new double[6];
      botpose_wpired = new double[6];
      botpose_wpiblue = new double[6];
      camerapose_robotspace = new double[6];
      targets_Retro = new LimelightTarget_Retro[0];
      targets_Fiducials = new LimelightTarget_Fiducial[0];
      targets_Classifier = new LimelightTarget_Classifier[0];
      targets_Detector = new LimelightTarget_Detector[0];
      targets_Barcode = new LimelightTarget_Barcode[0];
    }
  }

  // TODO: We are also missing some of the fluidial stuff
  public static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) {
      return new Pose2d();
    }
    Translation2d tran2d = new Translation2d(inData[0], inData[1]);
    Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
    return new Pose2d(tran2d, r2d);
  }

  public static Pose3d toPose3D(double[] inData) {
    if (inData.length < 6) {
      return new Pose3d();
    }
    return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(
            Units.degreesToRadians(inData[3]),
            Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5])));
  }

  /** Represents a 3D Pose Estimate. */
  // TODO: Consider encompassing PoseObservation within here, or combining PoseEstimate and
  // RawFiducial
  public static class PoseEstimate {
    public Pose2d pose;
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;

    // TODO: check for limelight tomfoolery with RawFiducial
    public RawFiducial[] rawFiducials;
    public boolean isMegaTag2;

    /** Instantiates a PoseEstimate object with default values */
    public PoseEstimate() {
      this.pose = new Pose2d();
      this.timestampSeconds = 0;
      this.latency = 0;
      this.tagCount = 0;
      this.tagSpan = 0;
      this.avgTagDist = 0;
      this.avgTagArea = 0;
      this.rawFiducials = new RawFiducial[] {};
      this.isMegaTag2 = false;
    }

    public PoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        double latency,
        int tagCount,
        double tagSpan,
        double avgTagDist,
        double avgTagArea,
        // TODO: limelight tomfoolery with RawFiducial
        RawFiducial[] rawFiducials,
        boolean isMegaTag2) {

      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.latency = latency;
      this.tagCount = tagCount;
      this.tagSpan = tagSpan;
      this.avgTagDist = avgTagDist;
      this.avgTagArea = avgTagArea;
      this.rawFiducials = rawFiducials;
      this.isMegaTag2 = isMegaTag2;
    }

    public boolean isValid() {
      return pose != null && tagCount > 0;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) return true;
      if (obj == null || getClass() != obj.getClass()) return false;
      PoseEstimate that = (PoseEstimate) obj;
      // We don't compare the timestampSeconds as it isn't relevant for equality and makes
      // unit testing harder
      return Double.compare(that.latency, latency) == 0
          && tagCount == that.tagCount
          && Double.compare(that.tagSpan, tagSpan) == 0
          && Double.compare(that.avgTagDist, avgTagDist) == 0
          && Double.compare(that.avgTagArea, avgTagArea) == 0
          && pose.equals(that.pose)
          && Arrays.equals(rawFiducials, that.rawFiducials);
    }
  }

  // TODO: Consider altering this
  public record PoseObservation(PoseEstimate poseEstimate, RawFiducial[] rawFiducials) {

    public PoseObservation() {
      this(new PoseEstimate(), new RawFiducial[] {});
    }

    public boolean isValid() {
      return poseEstimate != null
          && poseEstimate.isValid()
          && rawFiducials != null
          && rawFiducials.length > 0;
    }

    @Override
    public String toString() {
      if (!isValid()) {
        return "Invalid Pose Observation data.";
      }

      StringBuilder sb = new StringBuilder("Pose Estimate Information:\n");
      sb.append(poseEstimate.toString().indent(2));
      sb.append("\nRaw Fiducials Details:\n");

      for (int i = 0; i < rawFiducials.length; i++) {
        sb.append(String.format(" Fiducial #%d:%n", i + 1));
        sb.append(rawFiducials[i].toString().indent(4));
      }

      return sb.toString();
    }
  }
}
