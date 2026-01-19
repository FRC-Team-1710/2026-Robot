package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
  public static final Distance kFieldLength = Inches.of(651.2225);
  public static final Distance kFieldWidth = Inches.of(317.6875);

  public static final Distance kBumpWidth = Inches.of(73.08122);
  public static final Distance kBumpDistanceFromWall = Inches.of(62.373750);

  public static final Distance kTrenchWidth = Inches.of(50.34375);

  public static final Distance kBumpCenterYFromFieldCenter =
      kFieldWidth.div(2).minus(kBumpDistanceFromWall.plus(kBumpWidth.div(2)));
  public static final Distance kTrenchCenterYFromFieldCenter =
      kFieldWidth.div(2).minus(kTrenchWidth.div(2));

  public static final Distance kStartingLineDistance = Inches.of(158.6);
}
