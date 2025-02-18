// package frc.robot.commands;

// import java.util.List;

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.VisionSubsystem;

// public class DriveToPoseLimelight extends Command {
//   private final CommandSwerveDrivetrain drivetrain;
//   private final VisionSubsystem vision;
  
//   // The holonomic controller from PathPlanner with PID constants for translation and rotation.
//   private final PPHolonomicDriveController controller;
  
//   // The generated path
//   private com.pathplanner.lib.path.PathPlannerPath path;
//   private double totalTime;
  
//   // Timer for following the path.
//   private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

//   /**
//    * Constructs a DriveToPoseLimelight command.
//    *
//    * @param drivetrain The drivetrain subsystem.
//    * @param vision     The vision subsystem.
//    */
//   public DriveToPoseLimelight(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
//     this.drivetrain = drivetrain;
//     this.vision = vision;
    
//     // Create the PPHolonomicDriveController with your desired PID constants.
//     // These constants are from your snippet.
//     controller = new PPHolonomicDriveController(
//         new com.pathplanner.lib.config.PIDConstants(10, 0, 0),  // translation PID
//         new com.pathplanner.lib.config.PIDConstants(7, 0, 0)    // rotation PID
//     );
    
//     addRequirements(drivetrain, vision);
//   }

//   @Override
//   public void initialize() {
//     // Retrieve the latest vision target pose (non-null expected; if null, cancel the command)
//     Pose2d targetPose = vision.getLastVisionPose();
//     if(targetPose == null) {
//       cancel();
//       return;
//     }
    
//     // Get the robot's current pose.
//     Pose2d currentPose = drivetrain.getCurrentPose();
    
//     // Compute the travel direction (non-holonomic rotation) from current pose to target pose.
//     Rotation2d travelDirection = Rotation2d.fromRadians(
//       Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX())
//     );
    
//     // For waypoints, set the rotation component to be the travel direction (non-holonomic).
//     Pose2d startWaypoint = new Pose2d(currentPose.getTranslation(), travelDirection);
//     Pose2d endWaypoint = new Pose2d(targetPose.getTranslation(), travelDirection);
    
//     // Create a list of waypoints from the two poses.
//     List<Pose2d> waypoints = List.of(startWaypoint, endWaypoint);
    
//     // Define path constraints (max velocity, acceleration, etc.)
//     // (Adjust these values as needed.)
//     PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    
//     // Create the path. For an on-the-fly path, the ideal starting state can be null.
//     // For the goal, we set a holonomic rotation from the target pose.
//     path = new PathPlannerPath(
//         waypoints,
//         constraints,
//         null,
//         new GoalEndState(0.0, targetPose.getRotation()) // holonomic rotation at the end can be set to the vision target's rotation.
//     );
    
//     totalTime = path.getTotalTimeSeconds();
//     timer.reset();
//     timer.start();
//   }

//   @Override
//   public void execute() {
//     // Get the current time along the trajectory.
//     double t = timer.get();
    
//     // If the path is complete, do nothing.
//     if (t > totalTime) {
//       return;
//     }
    
//     // Sample the desired state from the path at time t.
//     // Assume getState(t) returns an object containing the desired pose and holonomic rotation.
//     var state = path.getState(t);
//     Pose2d desiredPose = state.pose;
//     Rotation2d desiredHolonomicRotation = state.holonomicRotation;
    
//     // Use the PPHolonomicDriveController to compute chassis speeds.
//     var chassisSpeeds = controller.calculate(drivetrain.getCurrentPose(), desiredPose, desiredHolonomicRotation);
    
//     // Command the drivetrain using field-centric control.
//     drivetrain.setControl(
//         new SwerveRequest.FieldCentric()
//             .withVelocityX(chassisSpeeds.vxMetersPerSecond)
//             .withVelocityY(chassisSpeeds.vyMetersPerSecond)
//             .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond)
//     );
//   }

//   @Override
//   public boolean isFinished() {
//     return timer.hasElapsed(totalTime);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.setControl(
//       new SwerveRequest.FieldCentric()
//       .withVelocityX(0)
//       .withVelocityY(0)
//       .withRotationalRate(0));
//     timer.stop();
//   }
// }