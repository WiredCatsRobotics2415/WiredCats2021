package frc.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystems.SwerveDrive;

public class AutoCircle extends SequentialCommandGroup {

    public AutoCircle(SwerveDrive swerveDrive) {
        final PathPlannerTrajectory trajectory = PathPlanner.loadPath("PIDTest", 1.5, 1.5, false);
        // setting up the pid controllers (theta for rotation) (these pid values need
        // tuning prob)
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, Constants.THETA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController xController = new PIDController(0.5, 0, 0);
        PIDController yController = new PIDController(0.5, 0, 0);

        SequentialCommandGroup swerveControllerCommand = new PPSwerveControllerCommand(trajectory, swerveDrive::getPose,
                Constants.swerveKinematics, xController, yController, thetaController, swerveDrive::setModuleStates,
                swerveDrive).andThen(new InstantCommand(() -> swerveDrive.drive(0, 0, 0)));
        Pose2d newPose = new Pose2d(trajectory.getInitialPose().getTranslation(), new Rotation2d(0));
        addCommands(new InstantCommand(() -> swerveDrive.resetOdometry(newPose)), swerveControllerCommand);
    }
}
