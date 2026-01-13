// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Subsystems;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;

@Logged
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
  // top
  // speed
  private double MaxAngularRate = RotationsPerSecond.of(1)
      .in(RadiansPerSecond); // 1 of a rotation per second max angular velocity

  /* Configure field-centric driving (forward is always away from driver) */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(
          SteerRequestType.MotionMagicExpo); // Smooth steering with MotionMagic

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Create subsystems (uses simulated versions when running in simulation) */
  private final Superstructure superstructure = new Superstructure();

  public RobotContainer() {
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
    return Commands.none();
  }

  public Vector<N2> rescaleTranslation(double x, double y) {
    Vector<N2> scaledJoyStick = VecBuilder.fill(x, y);
    scaledJoyStick = MathUtil.applyDeadband(scaledJoyStick, 0.1);
    return MathUtil.copyDirectionPow(scaledJoyStick, 2);
  }

  public double rescaleRotation(double rotation) {
    return Math.copySign(MathUtil.applyDeadband(rotation, 1), 2);
  }

  public HashMap<Subsystems, Pair<Runnable, Time>> getAllSubsystems() {
    HashMap<Subsystems, Pair<Runnable, Time>> map = new HashMap<>();
    map.put(Subsystems.Superstructure, new Pair<Runnable, Time>(superstructure::periodic, Milliseconds.of(20)));
    map.put(Subsystems.Drive, new Pair<Runnable, Time>(drivetrain::periodic, Milliseconds.of(20)));
    return map;
  }
}
