package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Field positions and waypoints as Pose2d constants for autonomous and teleoperated use. */
public class Waypoints {

  // Helper method to make pose creation cleaner
  private static Pose2d pose(double x, double y, double rotationDegrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(rotationDegrees));
  }

  // ==================== Starting Positions ====================
  public static final Pose2d START_LEFT = pose(7.150, 6.050, -120);
  public static final Pose2d START_CENTER = pose(7.150, 4.050, 0);
  public static final Pose2d START_RIGHT = pose(7.150, 2.050, 120);

  // ==================== Game Piece Intake Positions ====================
  // Actual intake positions (where game pieces are located)
  public static final Pose2d INTAKE_1 = pose(1.690, 7.374, -60);
  public static final Pose2d INTAKE_2 = pose(1.275, 7.074, -60);
  public static final Pose2d INTAKE_3 = pose(0.697, 6.645, -60);

  // Approach positions (safe distance before final intake position)
  public static final Pose2d INTAKE_1_APPROACH = pose(2.9, 7.23, -176.8);
  public static final Pose2d INTAKE_2_APPROACH = pose(3.5, 7.074, 0);
  public static final Pose2d INTAKE_3_APPROACH = pose(3.5, 7.074, 0);

  // Endpoint positions (drive target during vision search)
  public static final Pose2d INTAKE_1_ENDPOINT = pose(1.0, 6.64, -165);
  public static final Pose2d INTAKE_2_ENDPOINT = pose(1.275, 7.074, -60);
  public static final Pose2d INTAKE_3_ENDPOINT = pose(0.697, 6.645, -60);

  // ==================== Scoring Positions ====================
  public static final Pose2d SCORE_A = pose(3.161, 4.2, 0);
  public static final Pose2d SCORE_B = pose(3.161, 3.864, 0);
  public static final Pose2d SCORE_C = pose(3.671, 2.954, 60);
  public static final Pose2d SCORE_D = pose(3.969, 2.778, 60);
  public static final Pose2d SCORE_E = pose(5.022, 2.954, 120);
  public static final Pose2d SCORE_F = pose(5.303, 2.778, 120);
  public static final Pose2d SCORE_G = pose(5.832, 3.864, 180);
  public static final Pose2d SCORE_H = pose(5.832, 4.2, 180);
  public static final Pose2d SCORE_I = pose(5.303, 5.100, -120);
  public static final Pose2d SCORE_J = pose(5.022, 5.255, -120);
  public static final Pose2d SCORE_K = pose(3.969, 5.255, -60);
  public static final Pose2d SCORE_L = pose(3.671, 5.100, -60);

  // ==================== Navigation Waypoints ====================
  public static final Pose2d INT_LEFT = pose(5, 6.323, 0);
  public static final Pose2d INT_RIGHT = pose(5, 1.826, 0);
  public static final Pose2d MIDFIELD_CENTER = pose(8.27, 4.1, 0);

  // ==================== Testing Positions ====================
  public static final Pose2d TEST_1 = pose(2.82, 4.02, 180);
  public static final Pose2d TEST_2 = pose(1.5, 4.85, 0);
  public static final Pose2d TEST_3 = pose(1.5, 3.15, 0);

  private Waypoints() {
    // Utility class - prevent instantiation
  }
}
