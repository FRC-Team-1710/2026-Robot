package frc.robot.subsystems.vision.FuelDetection;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.*;

public class FuelFieldMap {
  public static final double MATCH_DISTANCE_M = 1.5;
  public static final double TRACK_EXPIRY_SECONDS = 10.0;

  private final List<FuelPosition> tracks = new ArrayList<>();

  public FuelFieldMap() {}

  public void updateTracks(List<Pose2d> detections) {
    for (FuelPosition t : tracks) t.markNotSeen();

    boolean[] claimed = new boolean[tracks.size()];

    for (Pose2d detection : detections) {
      int bestIdx = -1;
      double bestDist = MATCH_DISTANCE_M;

      for (int i = 0; i < tracks.size(); i++) {
        if (claimed[i]) continue;
        double d = detection.getTranslation().getDistance(tracks.get(i).getPose().getTranslation());
        if (d < bestDist) {
          bestDist = d;
          bestIdx = i;
        }
      }

      if (bestIdx >= 0) {
        claimed[bestIdx] = true;
        tracks.get(bestIdx).update(detection);
      } else {
        tracks.add(new FuelPosition(detection));
      }
    }

    // Expire stale tracks
    tracks.removeIf(t -> t.getAge() > TRACK_EXPIRY_SECONDS);
  }

  public List<FuelPosition> getTracks() {
    return tracks;
  }

  public void clearAllTracks() {
    tracks.clear();
    FuelPosition.resetIdCounter();
  }
}
