package frc.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystems.SwerveDrive;

public class AutoCircle extends SequentialCommandGroup {

    public AutoCircle(SwerveDrive swerveDrive) throws IOException {
        //basically java is dumb and requires exception handling for paths
        //but you can't try-catch this bc lambda expressions require the trajectory to be final
        //so the solution is to say autocircle throws the exception and catch it in the robot init
        Path filePath = Filesystem.getDeployDirectory().toPath().resolve("SimpleRotationTest.wpilib.json");
        final Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(filePath);
        //setting up the pid controllers (theta for rotation) (these pid values need tuning prob)
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, Constants.THETA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveDrive::getPose,
                Constants.swerveKinematics, xController, yController, thetaController, swerveDrive::setModuleStates,
                swerveDrive);
        addCommands(new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
                new ParallelCommandGroup(swerveControllerCommand, Robot.intake.getIntakeCommand()), Robot.intake.getIntakeCommand());
    }
}
