package frc.robot.constants;

import static edu.wpi.first.apriltag.AprilTagFields.kDefaultField;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.Mode.CurrentMode;
import java.io.IOException;

/** Field constants RELATIVE TO THE BLUE ALLIANCE (frc coordinate system) */
public class FieldConstants {
  public static final Distance kFieldLength = Inches.of(651.2225);
  public static final Distance kFieldWidth = Inches.of(317.6875);

  public static final Distance kBumpWidth = Inches.of(73.0);
  public static final Distance kBumpDepth = Inches.of(47.0);
  public static final Distance kBumpDistanceFromWall = Inches.of(62.343750);
  public static final Distance kBumpDistanceFromDS = Inches.of(182.11125);

  public static final Distance kTrenchWidth = Inches.of(50.34375);

  public static final Distance kHubWidth = Inches.of(47.0);
  public static final Distance kMaxHubWidth = Inches.of(58.435559);
  public static final Distance kMaxHubDistanceFromDS = Inches.of(215.605208);

  public static final Distance kBumpCenterYFromFieldCenter =
      kFieldWidth.div(2).minus(kBumpDistanceFromWall.plus(kBumpWidth.div(2)));
  public static final Distance kTrenchCenterYFromFieldCenter =
      kFieldWidth.div(2).minus(kTrenchWidth.div(2));

  public static final Distance kStartingLineDistance = Inches.of(157.61125);

  public static final Translation2d kFieldCenter =
      new Translation2d(kFieldLength.div(2), kFieldWidth.div(2));

  public static final Translation3d kHubCenterBlue =
      new Translation3d(Inches.of(182.112411), kFieldWidth.div(2), Inches.of(72.0));
  public static final Translation3d kHubCenterRed =
      new Translation3d(
          kFieldLength.minus(kHubCenterBlue.getMeasureX()), kFieldWidth.div(2), Inches.of(72.0));

  public static final Distance kDepotWidth = Inches.of(42);
  public static final Distance kDepotDepth = Inches.of(24);
  public static final Distance kDepotDistanceFromWall = Inches.of(213.84375);

  public static final Distance kHexagonRadius = Inches.of(20.9659045).times(2 / Math.sqrt(3));
  public static final Translation3d[] kHexagonBlue = {
    kHubCenterBlue.plus(
        new Translation3d(new Translation2d(kHexagonRadius.in(Meters), Rotation2d.k180deg))),
    kHubCenterBlue.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.k180deg.plus(Rotation2d.fromDegrees(60))))),
    kHubCenterBlue.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.k180deg.plus(Rotation2d.fromDegrees(120))))),
    kHubCenterBlue.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.k180deg.plus(Rotation2d.fromDegrees(180))))),
    kHubCenterBlue.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.k180deg.plus(Rotation2d.fromDegrees(240))))),
    kHubCenterBlue.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.k180deg.plus(Rotation2d.fromDegrees(300)))))
  };
  public static final Translation3d[] kHexagonRed = {
    kHubCenterRed.plus(
        new Translation3d(new Translation2d(kHexagonRadius.in(Meters), Rotation2d.kZero))),
    kHubCenterRed.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.kZero.minus(Rotation2d.fromDegrees(60))))),
    kHubCenterRed.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.kZero.minus(Rotation2d.fromDegrees(120))))),
    kHubCenterRed.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.kZero.minus(Rotation2d.fromDegrees(180))))),
    kHubCenterRed.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.kZero.minus(Rotation2d.fromDegrees(240))))),
    kHubCenterRed.plus(
        new Translation3d(
            new Translation2d(
                kHexagonRadius.in(Meters), Rotation2d.kZero.minus(Rotation2d.fromDegrees(300)))))
  };

  public static final AprilTagFieldLayout kAprilTags;

  static {
    try {
      if (Mode.currentMode == CurrentMode.SIM) {
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

  public static final Distance kOutpostCenterFromWall =
      Inches.of(49.25).div(2).plus(Inches.of(1.59375));

  public static final Distance kRobotLength = Inches.of(46);

  /** Trench + Bump = Tump */
  public class Tumps {
    public enum BlueTump {
      BLUE_LEFT_BUMP(
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).plus(kBumpCenterYFromFieldCenter.minus(kBumpWidth.div(2)))),
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).plus(kBumpCenterYFromFieldCenter.plus(kBumpWidth.div(2))))),
      BLUE_RIGHT_BUMP(
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).minus(kBumpCenterYFromFieldCenter.minus(kBumpWidth.div(2)))),
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).minus(kBumpCenterYFromFieldCenter.plus(kBumpWidth.div(2))))),
      BLUE_LEFT_TRENCH(
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).plus(kTrenchCenterYFromFieldCenter.minus(kTrenchWidth.div(2)))),
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).plus(kTrenchCenterYFromFieldCenter.plus(kTrenchWidth.div(2))))),
      BLUE_RIGHT_TRENCH(
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).minus(kTrenchCenterYFromFieldCenter.minus(kTrenchWidth.div(2)))),
          new Translation2d(
              kBumpDistanceFromDS,
              kFieldWidth.div(2).minus(kTrenchCenterYFromFieldCenter.plus(kTrenchWidth.div(2)))));

      public final Translation2d translation1;
      public final Translation2d translation2;
      public final Translation2d average;

      BlueTump(Translation2d translation1, Translation2d translation2) {
        this.translation1 = translation1;
        this.translation2 = translation2;
        this.average = translation1.plus(translation2).div(2);
      }

      public static BlueTump getClosest(Translation2d translation) {
        double closestDistance = Double.MAX_VALUE;
        BlueTump closestTump = BLUE_LEFT_BUMP;

        for (BlueTump tump : BlueTump.values()) {
          double distance = translation.getDistance(tump.average);
          if (distance < closestDistance) {
            closestDistance = distance;
            closestTump = tump;
          }
        }

        return closestTump;
      }
    }
  }
}
