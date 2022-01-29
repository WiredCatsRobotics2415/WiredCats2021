package frc.auto;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.subsystems.SwerveDrive;

public class AutoCircle extends SequentialCommandGroup {

    public AutoCircle(SwerveDrive swerveDrive) {
        TrajectoryConfig config = new TrajectoryConfig(Constants.AUTO_MAX_SPEED, Constants.AUTO_MAX_ACCELERATION)
                .setKinematics(Constants.swerveKinematics);

        Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
        waypoints.add(new Translation2d(0, 1));
        waypoints.add(new Translation2d(1, 1));
        waypoints.add(new Translation2d(1, 0));
        waypoints.add(new Translation2d(0, 0));
        waypoints.add(new Translation2d(0, 1));
        waypoints.add(new Translation2d(1, 1));
        waypoints.add(new Translation2d(1, 0));

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, Constants.THETA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveDrive::getPose,
                Constants.swerveKinematics, xController, yController, thetaController, swerveDrive::setModuleStates,
                swerveDrive);

        addCommands(new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand);
    }
}
