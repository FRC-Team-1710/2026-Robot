// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutoBuilder;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.autonomous.BLineRequest;
import frc.robot.constants.AutoConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSIM;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelSIM;
import frc.robot.subsystems.vision.LimelightSubsystem;

/**
 * RobotContainer - Sets up all the robot's parts and controls.
 *
 * <p>This class handles:
 * <ul>
 *   <li>Creating all subsystems (drive, arm, flywheel, vision, etc.)
 *   <li>Setting up controller buttons
 *   <li>Building autonomous routines
 *   <li>Setting default actions for each subsystem
 * </ul>
 *
 * <p>The robot can run in two modes:
 * <ul>
 *   <li><b>Real hardware:</b> Uses actual motors and sensors
 *   <li><b>Simulation:</b> Uses simulated physics for testing without a real robot
 * </ul>
 * The code automatically picks the right version.
 */
@Logged
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
  // top
  // speed
  private double MaxAngularRate =
      RotationsPerSecond.of(1)
          .in(RadiansPerSecond); // 1 of a rotation per second max angular velocity

  /* Configure field-centric driving (forward is always away from driver) */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(
              SteerRequestType.MotionMagicExpo); // Smooth steering with MotionMagic

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final AutoBuilder autoCommands = new AutoBuilder(
      drivetrain,
      new BLineRequest(
          new Constraints(AutoConstants.MAX_LINEAR_VELOCITY_MPS, AutoConstants.MAX_LINEAR_ACCELERATION_MPS2),
          new Constraints(AutoConstants.MAX_ANGULAR_VELOCITY_RAD_S, AutoConstants.MAX_ANGULAR_ACCELERATION_RAD_S2),
          new WheelForceCalculator(
              drivetrain.getModuleLocations(),
              Pounds.of(AutoConstants.ROBOT_MASS_LBS),
              KilogramSquareMeters.of(AutoConstants.MOMENT_OF_INERTIA_KG_M2))));

  /* Create subsystems (uses simulated versions when running in simulation) */
  public final Arm arm = RobotBase.isSimulation() ? new ArmSIM() : new Arm();
  public final Flywheel flywheel = RobotBase.isSimulation() ? new FlywheelSIM() : new Flywheel();
  private final Superstructure superstructure = new Superstructure(arm, flywheel);

  // Vision camera for tracking robot position
  public final LimelightSubsystem limelight =
      new LimelightSubsystem("limelight", drivetrain);

  /* Autonomous mode selector */
  private final SendableChooser<Command> autoChooser;

  private final AutoRoutines autoRoutines;

  public RobotContainer() {

    // Set up autonomous routines
    autoChooser = new SendableChooser<>();
    autoRoutines = new AutoRoutines(autoCommands, superstructure);

    // Add autonomous mode options to dashboard
    autoChooser.addOption("Mobility Auto", autoRoutines.mobilityAuto());

    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // Controller axes: X = forward/backward, Y = left/right
    // (This is WPILib's standard coordinate system)
    drivetrain.setDefaultCommand(
        // Robot drives using joystick inputs by default
        drivetrain.applyRequest(
            () -> {
              Vector<N2> scaledInputs = rescaleTranslation(joystick.getLeftY(), joystick.getLeftX());
              return drive
                  .withVelocityX(-scaledInputs.get(0, 0) * MaxSpeed)
                  .withVelocityY(-scaledInputs.get(1, 0) * MaxSpeed)
                  .withRotationalRate(-rescaleRotation(joystick.getRightX()) * MaxAngularRate);
            }));

    joystick
        .start()
        .onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(Feet.of(0), Feet.of(0), Rotation2d.kZero))));
  }

  public Command getAutonomousCommand() {
    /* Return whichever autonomous mode was selected on the dashboard */
    return autoChooser.getSelected();
  }

  public Vector<N2> rescaleTranslation(double x, double y) {
    Vector<N2> scaledJoyStick = VecBuilder.fill(x, y);
    scaledJoyStick = MathUtil.applyDeadband(scaledJoyStick, 0.1);
    return MathUtil.copyDirectionPow(scaledJoyStick, 2);
  }

  public double rescaleRotation(double rotation){
    return Math.copySign(MathUtil.applyDeadband(rotation, 1), 2);
  }
}
