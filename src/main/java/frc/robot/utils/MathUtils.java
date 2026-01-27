package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

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

    Robot.telemetry().log("MathUtils/LinePenetrationT", t);
    Robot.telemetry().log("MathUtils/LinePenetrationU", u);

    return t > 0 && u >= 0 && u <= 1;
  }
}
