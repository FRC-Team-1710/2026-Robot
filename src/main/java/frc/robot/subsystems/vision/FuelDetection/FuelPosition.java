package frc.robot.subsystems.vision.FuelDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

/** Represents one tracked sphere across time. */
public class FuelPosition {

  private static int nextId = 0;

  /** Stable ID assigned at first detection. Never changes. */
  public final int id;

  /** Last known field-relative position. Updated when the ball is seen again. */
  private Pose2d pose;

  /** Whether the ball was seen in the most recent camera frame. */
  private boolean currentlyVisible;

  /** Timestamp (FPGA seconds) of the most recent observation. */
  private double lastSeenTimestamp;

  /** How many consecutive frames this track has been updated (confidence proxy). */
  private int hitCount;

  public FuelPosition(Pose2d initialPose) {
    this.id = nextId++;
    this.pose = initialPose;
    this.currentlyVisible = true;
    this.lastSeenTimestamp = Timer.getFPGATimestamp();
    this.hitCount = 1;
  }

  /** Update this track with a fresh detection. */
  public void update(Pose2d newPose) {
    this.pose = newPose;
    this.currentlyVisible = true;
    this.lastSeenTimestamp = Timer.getFPGATimestamp();
    this.hitCount++;
  }

  /** Mark the ball as not seen this frame (position is retained). */
  public void markNotSeen() {
    this.currentlyVisible = false;
  }

  public Pose2d getPose() {
    return pose;
  }

  public boolean isCurrentlyVisible() {
    return currentlyVisible;
  }

  public double getLastSeenTimestamp() {
    return lastSeenTimestamp;
  }

  public int getHitCount() {
    return hitCount;
  }

  /** Seconds since this ball was last observed. */
  public double getAge() {
    return Timer.getFPGATimestamp() - lastSeenTimestamp;
  }

  @Override
  public String toString() {
    return String.format(
        "TrackedSphere{id=%d, visible=%b, age=%.2fs, pos=(%.2f, %.2f)}",
        id, currentlyVisible, getAge(), pose.getX(), pose.getY());
  }

  public static void resetIdCounter() {
    nextId = 0;
  }
}
