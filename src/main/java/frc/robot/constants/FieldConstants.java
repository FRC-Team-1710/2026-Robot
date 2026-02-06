package frc.robot.constants;

import static edu.wpi.first.apriltag.AprilTagFields.kDefaultField;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.io.IOException;

/** Field constants RELATIVE TO THE BLUE ALLIANCE (frc coordinate system) */
public class FieldConstants {
  public static final Distance kFieldLength = Inches.of(651.2225);
  public static final Distance kFieldWidth = Inches.of(317.6875);

  public static final Distance kBumpWidth = Inches.of(73.08122);
  public static final Distance kBumpDepth = Inches.of(47);
  public static final Distance kBumpDistanceFromWall = Inches.of(62.373750);

  public static final Distance kTrenchWidth = Inches.of(50.34375);

  public static final Distance kBumpCenterYFromFieldCenter =
      kFieldWidth.div(2).minus(kBumpDistanceFromWall.plus(kBumpWidth.div(2)));
  public static final Distance kTrenchCenterYFromFieldCenter =
      kFieldWidth.div(2).minus(kTrenchWidth.div(2));

  public static final Distance kStartingLineDistance = Inches.of(158.6);

  public static final Translation2d kHubCenterBlue =
      new Translation2d(Inches.of(182.11125), kFieldWidth.div(2));

  public static final Distance kDepotWidth = Inches.of(42);
  public static final Distance kDepotDistanceFromWall = Inches.of(213.84375);

  public static final AprilTagFieldLayout kAprilTags;

  static {
    try {
      if (Mode.currentMode == Mode.currentMode.SIMULATION) {
        kAprilTags = AprilTagFieldLayout.loadFromResource(kDefaultField.m_resourceFile);
      } else {
        kAprilTags = AprilTagFieldLayout.loadField(kDefaultField);
        // AprilTagFieldLayout.loadFromResource(
        //     "src/main/java/frc/robot/utils/wpicalfields/practicefield.json");
      }
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // Add a translation2d to this subclass and automatically add it to the auto chooser
  public class AutoConstants {
    public static final Translation2d kRightTrenchEntrance =
        new Translation2d(kStartingLineDistance, kTrenchWidth.div(2));
    public static final Translation2d kRightTrenchExit =
        new Translation2d(kStartingLineDistance.plus(kBumpDepth), kTrenchWidth.div(2));

    public static final Translation2d kLeftTrenchEntrance =
        new Translation2d(kStartingLineDistance, kFieldWidth.minus(kTrenchWidth.div(2)));
    public static final Translation2d kLeftTrenchExit =
        new Translation2d(
            kStartingLineDistance.plus(kBumpDepth), kFieldWidth.minus(kTrenchWidth.div(2)));

    public static final Translation2d kRightBumpEntrance =
        new Translation2d(kStartingLineDistance, kBumpWidth.div(2).plus(kBumpDistanceFromWall));
    public static final Translation2d kRightBumpExit =
        new Translation2d(
            kStartingLineDistance.plus(kBumpDepth), kBumpWidth.div(2).plus(kBumpDistanceFromWall));

    public static final Translation2d kLeftBumpEntrance =
        new Translation2d(
            kStartingLineDistance,
            kFieldWidth.minus(kBumpWidth.div(2).plus(kBumpDistanceFromWall)));
    public static final Translation2d kLeftBumpExit =
        new Translation2d(
            kStartingLineDistance.plus(kBumpDepth),
            kFieldWidth.minus(kBumpWidth.div(2).plus(kBumpDistanceFromWall)));

    public static final Distance kOutpostCenterFromWall = Inches.of(47.5).div(2);

    public static final Distance kDepotCenterDistanceFromWall =
        kDepotDistanceFromWall.plus(kDepotWidth.div(2));
  }
}
