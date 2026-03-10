package frc.robot.subsystems.vision.FuelDetection;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.*;

public class FuelFieldMap {
  public static final double MATCH_DISTANCE_M = 1.5;
  public static final double TRACK_EXPIRY_SECONDS = 10.0;

  private final List<FuelPosition> tracks = new ArrayList<>();
  // Reused BitSet to avoid allocating a new boolean[] on every update
  private final java.util.BitSet claimed = new java.util.BitSet();

  public FuelFieldMap() {}

  public void updateTracks(List<Pose2d> detections) {
    for (FuelPosition t : tracks) t.markNotSeen();

    // Resize/reset reusable BitSet to current tracks size
    claimed.clear();
    // Note: BitSet grows automatically when setting bits beyond current length

    for (Pose2d detection : detections) {
      int bestIdx = -1;
      double bestDist = MATCH_DISTANCE_M;

      for (int i = 0; i < tracks.size(); i++) {
        if (claimed.get(i)) continue;
        // Use Translation2d distance which allocates less than creating temporaries here
        double d = detection.getTranslation().getDistance(tracks.get(i).getPose().getTranslation());
        if (d < bestDist) {
          bestDist = d;
          bestIdx = i;
        }
      }

      if (bestIdx >= 0) {
        claimed.set(bestIdx);
        tracks.get(bestIdx).update(detection);
      } else {
        tracks.add(new FuelPosition(detection));
      }
    }

    // Expire stale tracks. Cache current time once to avoid repeated Timer calls.
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    tracks.removeIf(t -> t.getAge(now) > TRACK_EXPIRY_SECONDS);
  }

  public List<FuelPosition> getTracks() {
    return tracks;
  }

  public void clearAllTracks() {
    tracks.clear();
    FuelPosition.resetIdCounter();
  }
}
