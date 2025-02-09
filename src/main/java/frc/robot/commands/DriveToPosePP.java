package frc.robot.commands;

import com.pathplanner.lib.path.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;

public class DriveToPosePP extends Command {
    private final Drive drive;
    private final Pose2d goalPose;
    private Command followTrajectoryCommand;

   public DriveToPosePP(Drive drive, Pose2d goalPose) {
    this.drive = drive;
    this.goalPose = goalPose;
    addRequirements(drive);
}

@Override
public void initialize() {
    Pose2d startingPose = drive.getPose();

    PathConstraints constraints = new PathConstraints(
        drive.getMaxLinearSpeedMetersPerSec(),
        2.0,
        2 * Math.PI,
        4 * Math.PI
    );

    LocalADStarAK adStar = new LocalADStarAK();
    adStar.setStartPosition(startingPose.getTranslation());
    adStar.setGoalPosition(goalPose.getTranslation());
    PathPlannerPath path = adStar.getCurrentPath(constraints, new GoalEndState(0.0, goalPose.getRotation()));

    // followTrajectoryCommand = new SwerveControllerCommand(
    //     path.getTrajectory(),
    //     drive::getPose,
    //     drive.getKinematics(),
    //     new PIDController(1.0, 0.0, 0.0),
    //     new PIDController(1.0, 0.0, 0.0),
    //     new PIDController(1.0, 0.0, 0.0),
    //     drive::setModuleStates,
    //     drive
    // );

    followTrajectoryCommand.initialize();
}

@Override
public void execute() {
    followTrajectoryCommand.execute();
}

@Override
public void end(boolean interrupted) {
    followTrajectoryCommand.end(interrupted);
}

@Override
public boolean isFinished() {
    return followTrajectoryCommand.isFinished();
}
}
