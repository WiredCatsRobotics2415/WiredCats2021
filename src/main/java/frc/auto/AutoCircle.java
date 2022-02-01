package frc.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystems.SwerveDrive;

public class AutoCircle extends SequentialCommandGroup {

    public AutoCircle(SwerveDrive swerveDrive) {
        final PathPlannerTrajectory trajectory = PathPlanner.loadPath("SimpleRotationTest", 3, 3, true);
        // setting up the pid controllers (theta for rotation) (these pid values need
        // tuning prob)
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, Constants.THETA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);

        SequentialCommandGroup swerveControllerCommand = new PPSwerveControllerCommand(trajectory, swerveDrive::getPose,
                Constants.swerveKinematics, xController, yController, thetaController, swerveDrive::setModuleStates,
                swerveDrive).andThen(new InstantCommand(() -> swerveDrive.drive(0, 0, 0)));
        addCommands(new InstantCommand(() -> swerveDrive.resetOdometry(trajectory.getInitialPose())),
                new ParallelCommandGroup(swerveControllerCommand,
                        new WaitCommand(trajectory.getTotalTimeSeconds() - 1)
                                .andThen(Robot.spindexer.getSpindexerCommand(), Robot.intake.getIntakeCommand())),
                new WaitCommand(1),
                new ParallelCommandGroup(Robot.intake.getIntakeCommand(), Robot.spindexer.getSpindexerCommand()));
    }
}
