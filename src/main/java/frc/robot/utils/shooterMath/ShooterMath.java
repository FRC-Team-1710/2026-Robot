// package frc.robot.utils.shooterMath;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.RotationsPerSecond;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Robot;
// import frc.robot.constants.Alliance;
// import frc.robot.constants.FieldConstants;
// import frc.robot.constants.ShooterConstants;
// import frc.robot.utils.MathUtils;

// /** Utility class for shooter-related mathematical calculations. */
// public class ShooterMath {

//   // --- Hardware Specs ---
//   private static final double kTopDiameter = 3.0;
//   private static final double kBottomDiameter = 2.0;
//   private static final double kCompression = 1.0;

//   // Backspin lift reduces required speed by ~12%
//   private static final double kLiftCoefficient = 0.12;

//   // --- Physics Constants ---
//   private static final double kG = 9.80665;
//   private static final double kGravityEffect = kG * (1.0 - kLiftCoefficient);
//   private static final double kTargetHeight =
//       Units.inchesToMeters(78.0) - ShooterConstants.kLEFT_SHOOTER_OFFSET.getZ();

//   // --- Robot State ---
//   public static Translation2d currentTranslation;
//   public static Translation2d relativeTranslation;
//   public static ChassisSpeeds currentSpeeds;

//   // Current solution
//   private static ShotSolution currentLeftSolution = new ShotSolution();
//   private static ShotSolution currentRightSolution = new ShotSolution();

//   // --- Memory Saving (not creating new objects each calculation) ---
//   private static double dist = 0;
//   private static double thetaGround = 0;
//   private static double v0Ground = 0;
//   private static double unitX = 0;
//   private static double unitY = 0;
//   private static double vRobLongitudinal = 0;
//   private static double vShotX_forward = 0;
//   private static double vShotX_lateral = 0;
//   private static double vShotY = 0;
//   private static double vShotX_horizontal_net = 0;
//   private static double thetaHoodDeg = 0;
//   private static double thetaHoodRad = 0;
//   private static double vExitTotal = 0;

//   public static class ShotSolution {
//     public AngularVelocity flywheelSpeed = RotationsPerSecond.of(0);
//     public Angle hoodAngle = Degrees.of(0);
//     public Rotation2d robotHeading = Rotation2d.kZero;
//     public boolean isLimited = false;
//   }

//   static {
//     SmartDashboard.putNumber("NUIBNIB123456", 0);
//   }

//   public static void calculate(ShotSolution solution) {

//     dist =
//         Math.sqrt(
//             relativeTranslation.getX() * relativeTranslation.getX()
//                 + relativeTranslation.getY() * relativeTranslation.getY());

//     // 1. Calculate the IDEAL GROUND trajectory (Apex shot)
//     thetaGround =
//         Math.atan((kTargetHeight + Math.sqrt(dist * dist + kTargetHeight * kTargetHeight)) /
// dist);
//     v0Ground =
//         Math.sqrt(
//             kGravityEffect
//                 * (kTargetHeight + Math.sqrt(dist * dist + kTargetHeight * kTargetHeight)));

//     // 3. Project Robot Velocity into Target-Relative Frame
//     unitX = relativeTranslation.getX() / dist;
//     unitY = relativeTranslation.getY() / dist;

//     // Speed toward/away from target
//     vRobLongitudinal =
//         (currentSpeeds.vxMetersPerSecond * unitX) + (currentSpeeds.vyMetersPerSecond * unitY);

//     // 4. Shooter must provide the missing components
//     // We need to provide v0x_total, but the robot already provides vRobLongitudinal
//     vShotX_forward = v0Ground * Math.cos(thetaGround) - vRobLongitudinal;
//     // We need 0 lateral ground speed, so we must counter vRobLateral exactly
//     vShotX_lateral =
//         -((currentSpeeds.vxMetersPerSecond * (-unitY)) + (currentSpeeds.vyMetersPerSecond *
// unitX));
//     vShotY = v0Ground * Math.sin(thetaGround);

//     // 5. Total Horizontal Velocity the shooter must produce
//     vShotX_horizontal_net = Math.sqrt(Math.pow(vShotX_forward, 2) + Math.pow(vShotX_lateral, 2));

//     // 6. Calculate Shooter Angles
//     thetaHoodRad = Math.atan2(vShotY, vShotX_horizontal_net);
//     vExitTotal = Math.sqrt(Math.pow(vShotX_horizontal_net, 2) + Math.pow(vShotY, 2));

//     // 7. Check Physical Hood Limits
//     thetaHoodDeg = Math.toDegrees(thetaHoodRad);
//     solution.isLimited = false;
//     if (thetaHoodDeg < 90 - ShooterConstants.HOOD_MAX
//         || thetaHoodDeg > 90 - ShooterConstants.HOOD_MIN) {
//       thetaHoodDeg =
//           Math.max(
//               90 - ShooterConstants.HOOD_MAX,
//               Math.min(90 - ShooterConstants.HOOD_MIN, thetaHoodDeg));
//       thetaHoodRad = Math.toRadians(thetaHoodDeg);

//       // Re-calculate exit velocity components with the new ground requirement
//       vShotX_forward =
//           (((dist / Math.cos(thetaHoodRad))
//                       * Math.sqrt(
//                           kGravityEffect / (2 * (dist * Math.tan(thetaHoodRad) -
// kTargetHeight))))
//                   * Math.cos(thetaHoodRad))
//               - vRobLongitudinal;
//       vExitTotal =
//           Math.sqrt(
//               Math.pow(Math.sqrt(Math.pow(vShotX_forward, 2) + Math.pow(vShotX_lateral, 2)), 2)
//                   + Math.pow(vShotY, 2));
//       solution.isLimited = true;
//     }

//     // 9. Final Robot Heading Calculation (Field-relative)
//     // Angle to target center + the lead angle compensation
//     solution.flywheelSpeed =
//         RotationsPerSecond.of(
//             (2.0 * vExitTotal * 39.37)
//                 / (Math.PI
//                     * ((kTopDiameter - (kCompression / 2.0))
//                         + (kBottomDiameter - (kCompression / 2.0)))));
//     solution.hoodAngle = Degrees.of(90 - thetaHoodDeg);
//     solution.robotHeading =
//         Rotation2d.fromRadians(
//             Math.atan2(-relativeTranslation.getY(), -relativeTranslation.getX())
//                 + Math.atan2(vShotX_lateral, vShotX_forward)
//                 + Math.PI);

//     // d = relativeTranslation.getNorm();

//     // // 1. Try for the "Ideal" Lowest RPM Angle (Apex Shot)
//     // idealTheta =
//     //     Math.toDegrees(
//     //         Math.atan(
//     //             (kTargetHeight + Math.sqrt(Math.pow(d, 2) + Math.pow(kTargetHeight, 2))) /
// d));

//     // // 2. Check Limits and Re-calculate if necessary (90- to convert from hood to trajectory)
//     // if (idealTheta > 90 - ShooterConstants.HOOD_MIN) {
//     //   finalThetaRad = Math.toRadians(90 - ShooterConstants.HOOD_MIN);
//     //   requiredV0 = solveForFixedAngle(d, finalThetaRad);
//     //   solution.isLimited = true;
//     // } else if (idealTheta < 90 - ShooterConstants.HOOD_MAX) {
//     //   finalThetaRad = Math.toRadians(90 - ShooterConstants.HOOD_MAX);
//     //   requiredV0 = solveForFixedAngle(d, finalThetaRad);
//     //   solution.isLimited = true;
//     // } else {
//     //   finalThetaRad = Math.toRadians(idealTheta);
//     //   requiredV0 =
//     //       Math.sqrt(
//     //           gravityEffect
//     //               * (kTargetHeight + Math.sqrt(Math.pow(d, 2) + Math.pow(kTargetHeight, 2))));
//     //   solution.isLimited = false;
//     // }

//     // // 3. Vector Compensation for Moving Robot
//     // unitX = relativeTranslation.getX() / d;
//     // unitY = relativeTranslation.getY() / d;

//     // // 4. Build Result
//     // solution.flywheelSpeed =
//     //     RotationsPerSecond.of(
//     //         (2.0
//     //                 * (requiredV0
//     //                     + (currentSpeeds.vxMetersPerSecond * unitX)
//     //                     - (currentSpeeds.vyMetersPerSecond * unitY))
//     //                 * 39.37)
//     //             / (Math.PI
//     //                 * ((kTopDiameter - (kCompression / 2.0))
//     //                     + (kBottomDiameter - (kCompression / 2.0)))));

//     // solution.hoodAngle = Degrees.of(90).minus(Radians.of(finalThetaRad));

//     // solution.robotHeading =
//     //     Rotation2d.fromRadians(
//     //         Math.atan2(-relativeTranslation.getY(), -relativeTranslation.getX())
//     //             - Math.atan2(
//     //                 ((currentSpeeds.vxMetersPerSecond * (-unitY))
//     //                     + (currentSpeeds.vyMetersPerSecond * unitX)),
//     //                 requiredV0)
//     //             + Math.PI);
//   }

//   // /** Calculates required velocity when the angle is fixed by physical limits. */
//   // private static double solveForFixedAngle(double d, double theta) {
//   //   // Projectile motion formula solving for V0:
//   //   // V0 = sqrt( (g * d^2) / (2 * cos^2(theta) * (d * tan(theta) - h)) )
//   //   double numerator = kGravityEffect * Math.pow(d, 2);
//   //   double denominator = 2 * Math.pow(Math.cos(theta), 2) * (d * Math.tan(theta) -
//   // kTargetHeight);

//   //   if (denominator <= 0) return 0; // Target is physically unreachable at this angle
//   //   return Math.sqrt(numerator / denominator);
//   // }

//   public static void updateRobotState(Translation2d translation, ChassisSpeeds speeds) {
//     currentSpeeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);

//     currentTranslation =
//         translation.plus(
//             ShooterConstants.kLEFT_SHOOTER_OFFSET
//                 .getTranslation()
//                 .toTranslation2d()
//                 .rotateBy(
//                     (Alliance.redAlliance
//                             ? FieldConstants.kHubCenterRed
//                             : FieldConstants.kHubCenterBlue)
//                         .minus(translation)
//                         .getAngle()));
//     relativeTranslation =
//         MathUtils.getClosestPointOnHexagon(
//                 currentTranslation, ShooterConstants.kLEFT_SHOOTER_OFFSET.getY())
//             .minus(currentTranslation);
//     calculate(currentLeftSolution);

//     currentTranslation =
//         translation.plus(
//             ShooterConstants.kRIGHT_SHOOTER_OFFSET
//                 .getTranslation()
//                 .toTranslation2d()
//                 .rotateBy(
//                     (Alliance.redAlliance
//                             ? FieldConstants.kHubCenterRed
//                             : FieldConstants.kHubCenterBlue)
//                         .minus(translation)
//                         .getAngle()));
//     relativeTranslation =
//         MathUtils.getClosestPointOnHexagon(
//                 currentTranslation, ShooterConstants.kRIGHT_SHOOTER_OFFSET.getY())
//             .minus(currentTranslation);
//     calculate(currentRightSolution);

//     Robot.telemetry().log("STRINGS", currentLeftSolution.hoodAngle.in(Degrees));
//     Robot.telemetry().log("Math/Left/Limited", currentLeftSolution.isLimited);
//     Robot.telemetry().log("Math/Left/Speed", currentLeftSolution.flywheelSpeed);
//     Robot.telemetry().log("Math/Left/Angle", currentLeftSolution.hoodAngle);
//     Robot.telemetry().log("Math/Left/Heading", currentLeftSolution.robotHeading,
// Rotation2d.struct);
//     Robot.telemetry().log("Math/Right/Limited", currentRightSolution.isLimited);
//     Robot.telemetry().log("Math/Right/Speed", currentRightSolution.flywheelSpeed);
//     Robot.telemetry().log("Math/Right/Angle", currentRightSolution.hoodAngle);
//     Robot.telemetry()
//         .log("Math/Right/Heading", currentRightSolution.robotHeading, Rotation2d.struct);
//     currentSpeeds = speeds;
//   }

//   public static ShotSolution getCurrentLeftSolution() {
//     return currentLeftSolution;
//   }

//   public static ShotSolution getCurrentRightSolution() {
//     return currentRightSolution;
//   }
// }
