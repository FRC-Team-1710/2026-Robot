// package frc.robot.utils;

// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
// import com.ctre.phoenix6.swerve.SwerveModule;
// import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.kinematics.SwerveModuleState;

// public class XDrive implements SwerveRequest {
//   public XDrive() {}

//   @Override
//   public StatusCode apply(
//       SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
//     for (int i = 0; i < modulesToApply.length; i++) {
//       modulesToApply[i].apply(
//           new ModuleRequest()
//               .withState(new SwerveModuleState(0, parameters.moduleLocations[i].getAngle()))
//               .withSteerRequest(
//                   "your steer request type: most likely Position but could be MotionMagicExpo"));
//     }
//     return StatusCode.OK;
//   }
// }

// Written for the best Owen in the world, TOOLCATS' Owen
