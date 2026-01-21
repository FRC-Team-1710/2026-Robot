package frc.robot.constants;

public class MotorID {

  /** Constants for the swerve drive motors */
  public static class Swerve {
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_LEFT_STEER = 2;
    public static final int FRONT_LEFT_ENCODER = 9;

    public static final int FRONT_RIGHT_DRIVE = 3;
    public static final int FRONT_RIGHT_STEER = 4;
    public static final int FRONT_RIGHT_ENCODER = 10;

    public static final int BACK_LEFT_DRIVE = 5;
    public static final int BACK_LEFT_STEER = 6;
    public static final int BACK_LEFT_ENCODER = 11;

    public static final int BACK_RIGHT_DRIVE = 7;
    public static final int BACK_RIGHT_STEER = 8;
    public static final int BACK_RIGHT_ENCODER = 12;
  }

  /** Constants for the intake motors */
  public static class Intake {
    public static final int INTAKE_MOTOR = 14;
    public static final int DEPLOYMENT_MOTOR = 15;
  }

  /** Constants for the indexer motors */
  public static class Indexer {
    public static final int INDEXER_MOTOR = 16;
  }
}
