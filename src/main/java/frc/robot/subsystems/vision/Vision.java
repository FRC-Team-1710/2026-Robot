package frc.robot.subsystems.vision;


import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import java.util.Optional;

@Logged
public class Vision extends SubsystemBase {
  private final String limelightName;
  private final CommandSwerveDrivetrain drivetrain;

  // Robot position from camera (using MegaTag)
  private Pose2d robotPose = new Pose2d(); // Where the camera thinks we are
  private double robotPoseTimestamp = 0.0; // When this measurement was taken
  private int tagCount = 0; // How many AprilTags the camera can see
  private double avgTagDistance = 0.0; // Average distance to visible tags (meters)
  private double ambiguity = 0.0; // Ambiguity value across all visible tags

  // System that records vision data for playback later
  private final HootAutoReplay autoReplay;

    public Vision(String limelightName, CommandSwerveDrivetrain drivetrain) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;

        // Set up data recording for the Limelight
        this.autoReplay =
            new HootAutoReplay()
                .withStruct( // Record robot position
                    "Limelight/" + limelightName + "/RobotPose",
                    Pose2d.struct,
                    () -> robotPose,
                    val -> robotPose = val.value)
                .withDouble( // Record timestamp
                    "Limelight/" + limelightName + "/RobotPoseTimestamp",
                    () -> robotPoseTimestamp,
                    val -> robotPoseTimestamp = val.value)
                .withInteger( // Record how many tags are visible
                    "Limelight/" + limelightName + "/TagCount",
                    () -> tagCount,
                    val -> tagCount = val.value.intValue())
                .withDouble( // Record average tag distance
                    "Limelight/" + limelightName + "/AvgTagDistance",
                    () -> avgTagDistance,
                    val -> avgTagDistance = val.value)
                .withDouble( // Record maximum ambiguity
                    "Limelight/" + limelightName + "/Ambiguity",
                    () -> ambiguity,
                    val -> ambiguity = val.value).withTimestampReplay();
    }

    @Override
    public void periodic() {
        
    }

}
