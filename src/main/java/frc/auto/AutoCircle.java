package frc.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.subsystems.SwerveDrive;

public class AutoCircle extends SequentialCommandGroup {

    public AutoCircle(SwerveDrive swerveDrive) {
        final PathPlannerTrajectory trajectory = PathPlanner.loadPath("SimpleRotationTest", 5.5, 3, true);
        // setting up the pid controllers (theta for rotation) (these pid values need
        // tuning prob)
        ProfiledPIDController thetaController = new ProfiledPIDController(0.3, 0, 0.1, Constants.THETA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController xController = new PIDController(0.7, 0, 0);
        PIDController yController = new PIDController(0.8, 0, 0);

        SequentialCommandGroup swerveControllerCommand = new PPSwerveControllerCommand(trajectory, swerveDrive::getPose,
                Constants.swerveKinematics, xController, yController, thetaController, swerveDrive::setModuleStates,
                swerveDrive).andThen(new InstantCommand(() -> swerveDrive.drive(0, 0, 0)));
        addCommands(new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose(), trajectory.getInitialState().holonomicRotation)), swerveControllerCommand);
    }
}
