package frc.robot.constants;

import static edu.wpi.first.apriltag.AprilTagFields.kDefaultField;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.Mode.CurrentMode;
import java.io.IOException;
import java.util.HashMap;

/** Field constants RELATIVE TO THE BLUE ALLIANCE (frc coordinate system) */
public class FieldConstants {
  public static final Distance kFieldLength = Inches.of(651.2225);
  public static final Distance kFieldWidth = Inches.of(317.6875);

  public static final Distance kBumpWidth = Inches.of(73.08122);
  public static final Distance kBumpDepth = Inches.of(47);
  public static final Distance kBumpDistanceFromWall = Inches.of(62.37375);
  public static final Distance kBumpDistanceFromDS = Inches.of(182.11125);

  public static final Distance kTrenchWidth = Inches.of(50.34375);

  public static final Distance kBumpCenterYFromFieldCenter =
      kFieldWidth.div(2).minus(kBumpDistanceFromWall.plus(kBumpWidth.div(2)));
  public static final Distance kTrenchCenterYFromFieldCenter =
      kFieldWidth.div(2).minus(kTrenchWidth.div(2));

  public static final Distance kStartingLineDistance = Inches.of(158.6);

  public static final Translation2d kHubCenterBlue =
      new Translation2d(Inches.of(182.11125), kFieldWidth.div(2));
  public static final Translation2d kHubCenterRed =
      new Translation2d(Inches.of(469.11125), kFieldWidth.div(2));

  public static final Distance kDepotWidth = Inches.of(42);
  public static final Distance kDepotLength = Inches.of(24);
  public static final Distance kDepotDistanceFromWall = Inches.of(213.84375);

  public static final AprilTagFieldLayout kAprilTags;

  static {
    try {
      if (Mode.currentMode == CurrentMode.SIMULATION) {
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

  public static final Distance kDepotCenterDistanceFromWall =
      kDepotDistanceFromWall.plus(kDepotWidth.div(2));

  public static final Distance kOutpostCenterFromWall = Inches.of(47.5).div(2);

  public static final Distance kRobotLength = Inches.of(46); // TODO: check measurement

  // Add a new value and automatically adds it to the auto chooser
  public static HashMap<String, Translation2d> AutoConstants() {
    HashMap<String, Translation2d> points = new HashMap<>();
    points.put(
        "Bump REn, ",
        new Translation2d(
            kStartingLineDistance.minus(kBumpDepth.div(2)),
            kBumpWidth.div(2).plus(kBumpDistanceFromWall)));
    points.put(
        "Bump REx, ",
        new Translation2d(
            kStartingLineDistance.plus(kBumpDepth).plus(kRobotLength.div(2)),
            kBumpWidth.div(2).plus(kBumpDistanceFromWall)));
    points.put(
        "Bump LEn, ",
        new Translation2d(
            kStartingLineDistance.minus(kBumpDepth.div(2)),
            kFieldWidth.minus(kBumpWidth.div(2).plus(kBumpDistanceFromWall))));
    points.put(
        "Bump LEx, ",
        new Translation2d(
            kStartingLineDistance.plus(kBumpDepth).plus(kRobotLength.div(2)),
            kFieldWidth.minus(kBumpWidth.div(2).plus(kBumpDistanceFromWall))));

    return points;
  }

  public class Bump {

    public enum BumpLocation {
      BLUE_LEFT(
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).plus(kBumpCenterYFromFieldCenter.minus(kBumpWidth.div(2)))),
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).plus(kBumpCenterYFromFieldCenter.plus(kBumpWidth.div(2))))),
      BLUE_RIGHT(
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).minus(kBumpCenterYFromFieldCenter.minus(kBumpWidth.div(2)))),
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).minus(kBumpCenterYFromFieldCenter.plus(kBumpWidth.div(2))))),
      RED_RIGHT(
          new Translation2d(
              kFieldLength.minus(kBumpDistanceFromDS),
              kFieldWidth.div(2).plus(kBumpCenterYFromFieldCenter.minus(kBumpWidth.div(2)))),
          new Translation2d(
              kFieldLength.minus(kBumpDistanceFromDS),
              kFieldWidth.div(2).plus(kBumpCenterYFromFieldCenter.plus(kBumpWidth.div(2))))),
      RED_LEFT(
          new Translation2d(
              kFieldLength.minus(kBumpDistanceFromDS),
              kFieldWidth.div(2).minus(kBumpCenterYFromFieldCenter.minus(kBumpWidth.div(2)))),
          new Translation2d(
              kFieldLength.minus(kBumpDistanceFromDS),
              kFieldWidth.div(2).minus(kBumpCenterYFromFieldCenter.plus(kBumpWidth.div(2)))));

      public final Translation2d translationInside;
      public final Translation2d translationOutside;
      public final Translation2d average;

      BumpLocation(Translation2d translationInside, Translation2d translationOutside) {
        this.translationInside = translationInside;
        this.translationOutside = translationOutside;
        this.average = translationInside.plus(translationOutside).div(2);
      }

      public static BumpLocation getClosest(Translation2d translation) {
        double closestDistance = Double.MAX_VALUE;
        BumpLocation closestBumpLocation = BLUE_LEFT;

        for (BumpLocation bumpLocation : BumpLocation.values()) {
          double distance = translation.getDistance(bumpLocation.translationInside);
          if (distance < closestDistance) {
            closestDistance = distance;
            closestBumpLocation = bumpLocation;
          }
        }

        return closestBumpLocation;
      }
    }
  }
}
