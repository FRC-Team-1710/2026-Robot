package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.Alliance;
import frc.robot.constants.FieldConstants;

public class MathUtils {

  /**
   * By using this method, you agree to the following terms and conditions:
   *
   * <p>1. The "Hallucination" Clause The AI may invent libraries, methods, or API endpoints that
   * sound perfectly plausible but do not actually exist. Just because it’s written in CamelCase
   * doesn't mean the compiler will recognize it.
   *
   * <p>2. Security & Vulnerability Disclaimer AI models are trained on massive datasets that
   * include "legacy" (read: old and insecure) code.
   *
   * <p>3. The "Context Blindness" Limitation it can see the snippet the user gave it, but it can’t
   * see the entire architecture.
   *
   * <p>4. Technical Debt Indemnification AI tends to be "eager to please," which can lead to
   * "spaghetti code" or overly verbose logic where a simple built-in function would suffice. This
   * can lead to increased technical debt and harder maintenance down the road.
   *
   * <p>5. The "Human-in-the-Loop" Requirement By using AI-generated code, the user acknowledges
   * that he/she is the Lead Engineer. The AI is the junior intern. The intern does the typing; the
   * Lead Engineer does the thinking, the testing, and will NOT take the ultimate responsibility.
   */
  public static boolean willPenetrateLine(
      Pose2d startPose, Translation2d point1, Translation2d point2) {
    var rotation = startPose.getRotation().unaryMinus();

    double dx = rotation.getSin();
    double dy = rotation.getCos();

    double x1 = point1.getX();
    double y1 = point1.getY();

    double x2 = point2.getX();
    double y2 = point2.getY();

    double sx = x2 - x1;
    double sy = y2 - y1;

    double denominator = (dx * sy) - (dy * sx);

    if (Math.abs(denominator) < 1e-10) {
      return false;
    }

    double t = (((x1 - startPose.getX()) * sy) - ((y1 - startPose.getY()) * sx)) / denominator;
    double u = (((startPose.getX() - x1) * dy) - ((startPose.getY() - y1) * dx)) / -denominator;

    return t > 0 && u >= 0 && u <= 1;
  }

  // Saving memory
  private static Translation3d[] tempHexagon;
  private static Translation2d tempCenter;
  private static Translation2d tempA;
  private static Translation2d tempB;
  private static Translation2d tempIntersect;

  /**
   * By using this method, you agree to the following terms and conditions:
   *
   * <p>1. The "Hallucination" Clause The AI may invent libraries, methods, or API endpoints that
   * sound perfectly plausible but do not actually exist. Just because it’s written in CamelCase
   * doesn't mean the compiler will recognize it.
   *
   * <p>2. Security & Vulnerability Disclaimer AI models are trained on massive datasets that
   * include "legacy" (read: old and insecure) code.
   *
   * <p>3. The "Context Blindness" Limitation it can see the snippet the user gave it, but it can’t
   * see the entire architecture.
   *
   * <p>4. Technical Debt Indemnification AI tends to be "eager to please," which can lead to
   * "spaghetti code" or overly verbose logic where a simple built-in function would suffice. This
   * can lead to increased technical debt and harder maintenance down the road.
   *
   * <p>5. The "Human-in-the-Loop" Requirement By using AI-generated code, the user acknowledges
   * that he/she is the Lead Engineer. The AI is the junior intern. The intern does the typing; the
   * Lead Engineer does the thinking, the testing, and will NOT take the ultimate responsibility.
   */
  public static Translation2d getClosestPointOnHexagon(Translation2d target, double offset) {
    tempHexagon = Alliance.redAlliance ? FieldConstants.kHexagonRed : FieldConstants.kHexagonBlue;
    tempCenter =
        (Alliance.redAlliance ? FieldConstants.kHubCenterRed : FieldConstants.kHubCenterBlue)
            .toTranslation2d()
            .plus(new Translation2d(0, offset));

    for (int i = 0; i < 6; i++) {
      tempA = tempHexagon[i].toTranslation2d();
      tempB = tempHexagon[(i + 1) % 6].toTranslation2d();

      tempIntersect = intersectSegments(tempCenter, target, tempA, tempB);
      if (tempIntersect != null) {
        return tempIntersect;
      }
    }
    // Should not happen if target is outside and center is inside
    return new Translation2d();
  }

  // Saving memory
  private static double tempDenominator;
  private static double tempUa;
  private static double tempUb;

  /**
   * By using this method, you agree to the following terms and conditions:
   *
   * <p>1. The "Hallucination" Clause The AI may invent libraries, methods, or API endpoints that
   * sound perfectly plausible but do not actually exist. Just because it’s written in CamelCase
   * doesn't mean the compiler will recognize it.
   *
   * <p>2. Security & Vulnerability Disclaimer AI models are trained on massive datasets that
   * include "legacy" (read: old and insecure) code.
   *
   * <p>3. The "Context Blindness" Limitation it can see the snippet the user gave it, but it can’t
   * see the entire architecture.
   *
   * <p>4. Technical Debt Indemnification AI tends to be "eager to please," which can lead to
   * "spaghetti code" or overly verbose logic where a simple built-in function would suffice. This
   * can lead to increased technical debt and harder maintenance down the road.
   *
   * <p>5. The "Human-in-the-Loop" Requirement By using AI-generated code, the user acknowledges
   * that he/she is the Lead Engineer. The AI is the junior intern. The intern does the typing; the
   * Lead Engineer does the thinking, the testing, and will NOT take the ultimate responsibility.
   */
  private static Translation2d intersectSegments(
      Translation2d p1, Translation2d p2, Translation2d p3, Translation2d p4) {
    tempDenominator =
        (p4.getY() - p3.getY()) * (p2.getX() - p1.getX())
            - (p4.getX() - p3.getX()) * (p2.getY() - p1.getY());

    if (tempDenominator == 0) return new Translation2d();

    tempUa =
        ((p4.getX() - p3.getX()) * (p1.getY() - p3.getY())
                - (p4.getY() - p3.getY()) * (p1.getX() - p3.getX()))
            / tempDenominator;
    tempUb =
        ((p2.getX() - p1.getX()) * (p1.getY() - p3.getY())
                - (p2.getY() - p1.getY()) * (p1.getX() - p3.getX()))
            / tempDenominator;

    if (tempUa >= 0 && tempUa <= 1 && tempUb >= 0 && tempUb <= 1) {
      return new Translation2d(
          p1.getX() + tempUa * (p2.getX() - p1.getX()),
          p1.getY() + tempUa * (p2.getY() - p1.getY()));
    }

    return null;
  }
}
