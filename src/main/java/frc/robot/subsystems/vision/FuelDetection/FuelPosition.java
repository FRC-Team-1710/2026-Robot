package frc.robot.subsystems.vision.FuelDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class FuelPositions {
  // TODO: Change detection system to account for 50 ms robot movement change
  public static final double SPHERE_DIAMETER_M = Units.inchesToMeters(4.91);
  public static final double SPHERE_RADIUS_M = SPHERE_DIAMETER_M / 2.0;

  /**
   * Max distance (metres) between a new detection and an existing track for them to be considered
   * the SAME ball.
   *
   * <p>Set this to roughly the max distance a ball could realistically travel between two
   * consecutive frames. ~0.5 m is a safe default for most FRC loops.
   */
  public static final double MATCH_DISTANCE_M = 1.5;

  /**
   * Optional: automatically forget a ball that hasn't been seen for this many seconds. Set to
   * {@link Double#MAX_VALUE} to remember balls forever (never expire).
   */
  public static final double TRACK_EXPIRY_SECONDS = 10.0;

  // ── Internal ──────────────────────────────────────────────────────────────

  private static final double MIN_DOWNWARD_RAY_Z = -0.01;

  // ── Nested tracked-ball model ─────────────────────────────────────────────

  /** Represents one tracked sphere across time. */
  public static class TrackedSphere {

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

    TrackedSphere(Pose2d initialPose) {
      this.id = nextId++;
      this.pose = initialPose;
      this.currentlyVisible = true;
      this.lastSeenTimestamp = Timer.getFPGATimestamp();
      this.hitCount = 1;
    }

    /** Update this track with a fresh detection. */
    void update(Pose2d newPose) {
      this.pose = newPose;
      this.currentlyVisible = true;
      this.lastSeenTimestamp = Timer.getFPGATimestamp();
      this.hitCount++;
    }

    /** Mark the ball as not seen this frame (position is retained). */
    void markNotSeen() {
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
  }
}
