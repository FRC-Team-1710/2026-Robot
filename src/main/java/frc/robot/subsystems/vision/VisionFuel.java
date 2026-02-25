package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.*;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionFuel extends SubsystemBase {

  // ── Physical constants ────────────────────────────────────────────────────

  public static final double SPHERE_DIAMETER_M = Units.inchesToMeters(4.91);
  public static final double SPHERE_RADIUS_M = SPHERE_DIAMETER_M / 2.0;

  // ── Tracker tuning ────────────────────────────────────────────────────────

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

    @Override
    public String toString() {
      return String.format(
          "TrackedSphere{id=%d, visible=%b, age=%.2fs, pos=(%.2f, %.2f)}",
          id, currentlyVisible, getAge(), pose.getX(), pose.getY());
    }
  }

  // ── State ─────────────────────────────────────────────────────────────────

  private final PhotonCamera camera;
  private Transform3d robotToCamera;
  private final Supplier<Pose2d> robotPoseSupplier;

  /** All currently tracked spheres, including those no longer visible. */
  private final List<TrackedSphere> tracks = new ArrayList<>();

  private final Field2d field = new Field2d();

  // ─────────────────────────────────────────────────────────────────────────
  // Constructor
  // ─────────────────────────────────────────────────────────────────────────

  /**
   * @param camera PhotonVision camera instance.
   * @param robotToCamera Transform3d from the robot centre to the camera optical centre.
   * @param robotPoseSupplier Supplier of the current robot {@link Pose2d} in field coordinates.
   */
  public VisionFuel(
      PhotonCamera camera, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
    this.camera = camera;
    this.robotToCamera = robotToCamera;
    this.robotPoseSupplier = robotPoseSupplier;

    SmartDashboard.putData("Sphere Vision/Field", field);

    SmartDashboard.putNumber("ima wah", 0);
  }

  // ── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    Pose2d robotPose = robotPoseSupplier.get();

    robotToCamera =
        new Transform3d(
            robotToCamera.getX(),
            robotToCamera.getY(),
            robotToCamera.getZ(),
            new Rotation3d(0, Units.degreesToRadians(SmartDashboard.getNumber("ima wah", 0)), 0));

    // 1. Compute raw detections from the camera
    List<Pose2d> detections = estimateSpherePoses(robotPose, camera, robotToCamera);

    // 2. Associate detections → existing tracks, update or spawn as needed
    updateTracks(detections, robotPose);

    // 3. Expire stale tracks (if expiry is enabled)
    tracks.removeIf(t -> t.getAge() > TRACK_EXPIRY_SECONDS);

    // 4. Dashboard output
    publishDashboard(robotPose);
  }

  // ── Track association ─────────────────────────────────────────────────────

  private void updateTracks(List<Pose2d> detections, Pose2d robotPose) {
    // Mark all tracks as "not seen yet this frame"
    for (TrackedSphere t : tracks) t.markNotSeen();

    // Track which existing tracks have already been claimed this frame
    boolean[] claimed = new boolean[tracks.size()];

    for (Pose2d detection : detections) {
      int bestIdx = -1;
      double bestDist = MATCH_DISTANCE_M; // only match if within threshold

      for (int i = 0; i < tracks.size(); i++) {
        if (claimed[i]) continue;
        double d = detection.getTranslation().getDistance(tracks.get(i).getPose().getTranslation());
        if (d < bestDist) {
          bestDist = d;
          bestIdx = i;
        }
      }

      if (bestIdx >= 0) {
        // ── Existing ball seen again (or moved) ───────────────────────
        claimed[bestIdx] = true;
        tracks.get(bestIdx).update(detection);
      } else {
        // ── Brand-new ball ────────────────────────────────────────────
        tracks.add(new TrackedSphere(detection));
      }
    }
    // Any unclaimed track stays in the list with currentlyVisible = false
    // and retains its last known pose automatically (no extra work needed).
  }

  // ── Ray-cast pose estimation ──────────────────────────

  public static List<Pose2d> estimateSpherePoses(
      Pose2d robotPose, PhotonCamera cam, Transform3d robotToCam) {
    List<Pose2d> results = new ArrayList<>();

    var photonResult = cam.getLatestResult();
    if (!photonResult.hasTargets()) return results;

    // Build camera Pose3d in field space
    Pose3d robotPose3d =
        new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));
    Pose3d cameraPose3d = robotPose3d.transformBy(robotToCam);

    double camX = cameraPose3d.getX();
    double camY = cameraPose3d.getY();
    double camZ = cameraPose3d.getZ();
    Rotation3d camRot = cameraPose3d.getRotation();

    if (camZ <= SPHERE_RADIUS_M) return results;

    for (PhotonTrackedTarget target : photonResult.getTargets()) {
      double yawRad =
          Units.degreesToRadians(
              (178.37 / 4)
                  - ((target.getMinAreaRectCorners().get(0).x
                              + target.getMinAreaRectCorners().get(1).x)
                          / 2)
                      / 960
                      * 178.37
                      / 2);
      double pitchRad =
          -Units.degreesToRadians(
              (177.74 / 4)
                  - ((target.getMinAreaRectCorners().get(1).y
                              + target.getMinAreaRectCorners().get(2).y)
                          / 2)
                      / 720
                      * 177.74
                      / 2);

      // Unit ray in camera frame (NWU: X-forward, Y-left, Z-up)
      double cosPitch = Math.cos(pitchRad);
      Translation3d rayCamera =
          new Translation3d(
              cosPitch * Math.cos(yawRad), cosPitch * Math.sin(yawRad), Math.sin(pitchRad));

      // Rotate ray into field frame
      Translation3d rayField = rayCamera.rotateBy(camRot);
      Robot.telemetry().log("rays", new Pose3d(rayField, new Rotation3d()), Pose3d.struct);
      double rayDz = rayField.getZ();

      if (rayDz > MIN_DOWNWARD_RAY_Z) continue;

      // Solve: camZ + t·rayDz = SPHERE_RADIUS_M  →  t = (SPHERE_RADIUS_M - camZ) / rayDz
      double t = (SPHERE_RADIUS_M - camZ) / rayDz;
      if (t <= 0.0) continue;

      double sphereX = camX + t * rayField.getX();
      double sphereY = camY + t * rayField.getY();

      // Heading: sphere faces the robot (useful for approach commands)
      double dx = robotPose.getX() - sphereX;
      double dy = robotPose.getY() - sphereY;
      results.add(new Pose2d(sphereX, sphereY, new Rotation2d(Math.atan2(dy, dx))));
    }

    return results;
  }

  // ── Dashboard ─────────────────────────────────────────────────────────────

  private void publishDashboard(Pose2d robotPose) {

    List<Pose2d> visiblePoses = new ArrayList<>();
    List<Pose2d> rememberedPoses = new ArrayList<>();

    for (TrackedSphere t : tracks) {
      if (t.isCurrentlyVisible()) visiblePoses.add(t.getPose());
      else rememberedPoses.add(t.getPose());
    }

    field.getObject("Spheres - Visible").setPoses(visiblePoses);
    field.getObject("Spheres - Remembered").setPoses(rememberedPoses);

    SmartDashboard.putNumber("Sphere Vision/Total Tracks", tracks.size());
    SmartDashboard.putNumber("Sphere Vision/Visible Now", visiblePoses.size());
    SmartDashboard.putNumber("Sphere Vision/Remembered", rememberedPoses.size());

    // Per-track detail
    for (TrackedSphere t : tracks) {
      String prefix = "Sphere Vision/Track " + t.id + "/";
      SmartDashboard.putBoolean(prefix + "visible", t.isCurrentlyVisible());
      SmartDashboard.putNumber(prefix + "x", t.getPose().getX());
      SmartDashboard.putNumber(prefix + "y", t.getPose().getY());
      SmartDashboard.putNumber(prefix + "age_sec", t.getAge());
      SmartDashboard.putNumber(prefix + "hits", t.getHitCount());
    }

    // Closest ball (visible or remembered)
    Pose2d closest = getClosestSphere(robotPose);
    if (closest != null) {
      SmartDashboard.putNumber("Sphere Vision/Closest/x", closest.getX());
      SmartDashboard.putNumber("Sphere Vision/Closest/y", closest.getY());
    }
  }

  /** All tracks — visible AND remembered. */
  public List<TrackedSphere> getAllTracks() {
    return Collections.unmodifiableList(tracks);
  }

  /** Only balls currently in camera view. */
  public List<TrackedSphere> getVisibleTracks() {
    return tracks.stream()
        .filter(TrackedSphere::isCurrentlyVisible)
        .collect(java.util.stream.Collectors.toList());
  }

  /** Balls that were seen before but are not currently visible. */
  public List<TrackedSphere> getRememberedTracks() {
    return tracks.stream()
        .filter(t -> !t.isCurrentlyVisible())
        .collect(java.util.stream.Collectors.toList());
  }

  /**
   * Closest sphere to the robot (visible or remembered). Returns {@code null} if no tracks exist.
   */
  public Pose2d getClosestSphere(Pose2d robotPose) {
    return tracks.stream()
        .map(TrackedSphere::getPose)
        .min(
            Comparator.comparingDouble(
                p -> p.getTranslation().getDistance(robotPose.getTranslation())))
        .orElse(null);
  }

  /**
   * Closest <em>currently visible</em> sphere to the robot. Returns {@code null} if nothing is
   * currently visible.
   */
  public Pose2d getClosestVisibleSphere(Pose2d robotPose) {
    return tracks.stream()
        .filter(TrackedSphere::isCurrentlyVisible)
        .map(TrackedSphere::getPose)
        .min(
            Comparator.comparingDouble(
                p -> p.getTranslation().getDistance(robotPose.getTranslation())))
        .orElse(null);
  }

  /** Manually clear all stored tracks (e.g. at the start of auto). */
  public void clearAllTracks() {
    tracks.clear();
  }
}
