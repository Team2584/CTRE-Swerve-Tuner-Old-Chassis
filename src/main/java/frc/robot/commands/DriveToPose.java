// // Copyright (c) 2025 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by an MIT-style
// // license that can be found in the LICENSE file at
// // the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import java.util.function.Supplier;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.util.GeomUtil;
// import frc.robot.util.TunableDashboardNumber;
// import org.littletonrobotics.junction.Logger;

// public class DriveToPose extends Command {
//   private static final TunableDashboardNumber drivekP = new TunableDashboardNumber("DriveToPose/DrivekP");
//   private static final TunableDashboardNumber drivekD = new TunableDashboardNumber("DriveToPose/DrivekD");
//   private static final TunableDashboardNumber thetakP = new TunableDashboardNumber("DriveToPose/ThetakP");
//   private static final TunableDashboardNumber thetakD = new TunableDashboardNumber("DriveToPose/ThetakD");
//   private static final TunableDashboardNumber driveMaxVelocity =
//       new TunableDashboardNumber("DriveToPose/DriveMaxVelocity");
//   private static final TunableDashboardNumber driveMaxVelocitySlow =
//       new TunableDashboardNumber("DriveToPose/DriveMaxVelocitySlow");
//   private static final TunableDashboardNumber driveMaxAcceleration =
//       new TunableDashboardNumber("DriveToPose/DriveMaxAcceleration");
//   private static final TunableDashboardNumber thetaMaxVelocity =
//       new TunableDashboardNumber("DriveToPose/ThetaMaxVelocity");
//   private static final TunableDashboardNumber thetaMaxVelocitySlow =
//       new TunableDashboardNumber("DriveToPose/ThetaMaxVelocitySlow");
//   private static final TunableDashboardNumber thetaMaxAcceleration =
//       new TunableDashboardNumber("DriveToPose/ThetaMaxAcceleration");
//   private static final TunableDashboardNumber driveTolerance =
//       new TunableDashboardNumber("DriveToPose/DriveTolerance");
//   private static final TunableDashboardNumber driveToleranceSlow =
//       new TunableDashboardNumber("DriveToPose/DriveToleranceSlow");
//   private static final TunableDashboardNumber thetaTolerance =
//       new TunableDashboardNumber("DriveToPose/ThetaTolerance");
//   private static final TunableDashboardNumber thetaToleranceSlow =
//       new TunableDashboardNumber("DriveToPose/ThetaToleranceSlow");
//   private static final TunableDashboardNumber ffMinRadius =
//       new TunableDashboardNumber("DriveToPose/FFMinRadius");
//   private static final TunableDashboardNumber ffMaxRadius =
//       new TunableDashboardNumber("DriveToPose/FFMinRadius");

//   static {
//     drivekP.initDefault(2.0);
//     drivekD.initDefault(0.0);
//     thetakP.initDefault(5.0);
//     thetakD.initDefault(0.0);
//     driveMaxVelocity.initDefault(Units.inchesToMeters(150.0));
//     driveMaxVelocitySlow.initDefault(Units.inchesToMeters(50.0));
//     driveMaxAcceleration.initDefault(Units.inchesToMeters(95.0));
//     thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
//     thetaMaxVelocitySlow.initDefault(Units.degreesToRadians(90.0));
//     thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
//     driveTolerance.initDefault(0.01);
//     driveToleranceSlow.initDefault(0.06);
//     thetaTolerance.initDefault(Units.degreesToRadians(1.0));
//     thetaToleranceSlow.initDefault(Units.degreesToRadians(3.0));
//     ffMinRadius.initDefault(0.2);
//     ffMaxRadius.initDefault(0.8);
//   }

//   private final Drive drive;
//   private final Supplier<Pose2d> target;

//   private final ProfiledPIDController driveController =
//       new ProfiledPIDController(
//           0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
//   private final ProfiledPIDController thetaController =
//       new ProfiledPIDController(
//           0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

//   private Translation2d lastSetpointTranslation = new Translation2d();
//   private double driveErrorAbs = 0.0;
//   private double thetaErrorAbs = 0.0;
//   private boolean running = false;

//   public DriveToPose(Drive drive, Supplier<Pose2d> target) {
//     this.drive = drive;
//     this.target = target;

//     // Enable continuous input for theta controller
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     addRequirements(drive);
//   }

//   @Override
//   public void initialize() {
//     Pose2d currentPose = drive.getPose();
//     ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
//     Translation2d linearFieldVelocity =
//         new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
//     driveController.reset(
//         currentPose.getTranslation().getDistance(target.get().getTranslation()),
//         Math.min(
//             0.0,
//             -linearFieldVelocity
//                 .rotateBy(
//                     target
//                         .get()
//                         .getTranslation()
//                         .minus(currentPose.getTranslation())
//                         .getAngle()
//                         .unaryMinus())
//                 .getX()));
//     thetaController.reset(
//         currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
//     lastSetpointTranslation = currentPose.getTranslation();
//   }

//   @Override
//   public void execute() {
//     running = true;

//     // Update from tunable numbers
//     if (driveMaxVelocity.hasChanged(hashCode())
//         || driveMaxVelocitySlow.hasChanged(hashCode())
//         || driveMaxAcceleration.hasChanged(hashCode())
//         || driveTolerance.hasChanged(hashCode())
//         || driveToleranceSlow.hasChanged(hashCode())
//         || thetaMaxVelocity.hasChanged(hashCode())
//         || thetaMaxVelocitySlow.hasChanged(hashCode())
//         || thetaMaxAcceleration.hasChanged(hashCode())
//         || thetaTolerance.hasChanged(hashCode())
//         || thetaToleranceSlow.hasChanged(hashCode())
//         || drivekP.hasChanged(hashCode())
//         || drivekD.hasChanged(hashCode())
//         || thetakP.hasChanged(hashCode())
//         || thetakD.hasChanged(hashCode())) {
//       driveController.setP(drivekP.get());
//       driveController.setD(drivekD.get());
//       driveController.setConstraints(
//           new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
//       driveController.setTolerance(driveTolerance.get());
//       thetaController.setP(thetakP.get());
//       thetaController.setD(thetakD.get());
//       thetaController.setConstraints(
//           new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
//       thetaController.setTolerance(thetaTolerance.get());
//     }

//     // Get current pose and target pose
//     Pose2d currentPose = drive.getPose();
//     Pose2d targetPose = target.get();

//     // Calculate drive speed
//     double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
//     double ffScaler =
//         MathUtil.clamp(
//             (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
//             0.0,
//             1.0);
//     driveErrorAbs = currentDistance;
//     driveController.reset(
//         lastSetpointTranslation.getDistance(targetPose.getTranslation()),
//         driveController.getSetpoint().velocity);
//     double driveVelocityScalar =
//         driveController.getSetpoint().velocity * ffScaler
//             + driveController.calculate(driveErrorAbs, 0.0);
//     if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
//     lastSetpointTranslation =
//         new Pose2d(
//                 targetPose.getTranslation(),
//                 currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
//             .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
//             .getTranslation();

//     // Calculate theta speed
//     double thetaVelocity =
//         thetaController.getSetpoint().velocity * ffScaler
//             + thetaController.calculate(
//                 currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
//     thetaErrorAbs =
//         Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
//     if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

//     // Command speeds
//     var driveVelocity =
//         new Pose2d(
//                 new Translation2d(),
//                 currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
//             .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
//             .getTranslation();
//     drive.runVelocity(
//         ChassisSpeeds.fromFieldRelativeSpeeds(
//             driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

//     // Log data
//     Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
//     Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
//     Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
//     Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
//     Logger.recordOutput(
//         "DriveToPose/Setpoint",
//         new Pose2d(
//             lastSetpointTranslation,
//             Rotation2d.fromRadians(thetaController.getSetpoint().position)));
//     Logger.recordOutput("DriveToPose/Goal", targetPose);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drive.stop();
//     running = false;
//     // Clear logs
//     Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
//     Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
//   }

//   /** Checks if the robot is stopped at the final pose. */
//   public boolean atGoal() {
//     return running && driveController.atGoal() && thetaController.atGoal();
//   }

//   /** Checks if the robot pose is within the allowed drive and theta tolerances. */
//   public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
//     return running
//         && Math.abs(driveErrorAbs) < driveTolerance
//         && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
//   }
// }