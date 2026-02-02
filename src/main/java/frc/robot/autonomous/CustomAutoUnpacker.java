package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.Path;
import java.util.ArrayList;
import java.util.List;

public class CustomAutoUnpacker {

  public static Path unpack(String packed) {
    String[] sections = packed.split("##");
    String constraintsPart = sections[0].substring("CONSTRAINTS|".length());
    String elementsPart = sections[1].substring("ELEMENTS|".length());

    // ===== UNPACK CONSTRAINTS =====
    Path.PathConstraints constraints = unpackConstraints(constraintsPart);

    // ===== UNPACK ELEMENTS =====
    List<Path.PathElement> elements = unpackElements(elementsPart);

    // ===== BUILD PATH =====
    Path path = new Path(elements.toArray(new Path.PathElement[0]));
    path.setPathConstraints(constraints);

    return path;
  }

  private static Path.PathConstraints unpackConstraints(String data) {
    String[] parts = data.split("::");

    // Base tolerances
    String[] base = parts[0].split(",");
    double endTransTol = Double.parseDouble(base[0]);
    double endRotTol = Double.parseDouble(base[1]);

    Path.PathConstraints c = new Path.PathConstraints();
    if (endTransTol != 0) {
      c.setEndTranslationToleranceMeters(endTransTol);
    }
    if (endRotTol != 0) {
      c.setEndRotationToleranceDeg(endRotTol);
    }

    // Ranged constraints
    c.setMaxVelocityMetersPerSec(unpackRanged(parts[1]));
    c.setMaxAccelerationMetersPerSec2(unpackRanged(parts[2]));
    c.setMaxVelocityDegPerSec(unpackRanged(parts[3]));
    c.setMaxAccelerationDegPerSec2(unpackRanged(parts[4]));

    return c;
  }

  private static Path.RangedConstraint[] unpackRanged(String block) {
    if (block.equals("EMPTY")) return new Path.RangedConstraint[0];

    String[] entries = block.split(";");
    ArrayList<Path.RangedConstraint> list = new ArrayList<>();

    for (String e : entries) {
      if (e.isBlank()) continue;
      String[] f = e.split("/");
      list.add(
          new Path.RangedConstraint(
              Double.parseDouble(f[0]), Integer.parseInt(f[1]), Integer.parseInt(f[2])));
    }

    return list.toArray(new Path.RangedConstraint[0]);
  }

  private static List<Path.PathElement> unpackElements(String data) {
    List<Path.PathElement> list = new ArrayList<>();
    String[] parts = data.split(";;");

    for (String part : parts) {
      if (part.isBlank()) continue;

      String[] typeAndData = part.split("\\|");
      String type = typeAndData[0];
      String[] fields = typeAndData[1].split(",");

      switch (type) {
        case "translation" -> {
          double x = Double.parseDouble(fields[0]);
          double y = Double.parseDouble(fields[1]);
          double radius = Double.parseDouble(fields[2]);
          list.add(
              new Path.TranslationTarget(new Translation2d(x, y), java.util.Optional.of(radius)));
        }

        case "event_trigger" -> {
          double t = Double.parseDouble(fields[0]);
          String key = fields[1];
          list.add(new Path.EventTrigger(t, key));
        }

        case "rotation" -> {
          double rotRad = Double.parseDouble(fields[0]);
          double t = Double.parseDouble(fields[1]);
          boolean profiled = Boolean.parseBoolean(fields[2]);
          list.add(new Path.RotationTarget(new Rotation2d(rotRad), t, profiled));
        }

        case "waypoint" -> {
          double x = Double.parseDouble(fields[0]);
          double y = Double.parseDouble(fields[1]);
          double radius = Double.parseDouble(fields[2]);
          double rotRad = Double.parseDouble(fields[3]);
          boolean profiled = Boolean.parseBoolean(fields[4]);

          Path.TranslationTarget tt =
              new Path.TranslationTarget(new Translation2d(x, y), java.util.Optional.of(radius));

          Path.RotationTarget rt = new Path.RotationTarget(new Rotation2d(rotRad), 1.0, profiled);

          list.add(new Path.Waypoint(tt, rt));
        }
      }
    }

    return list;
  }
}
