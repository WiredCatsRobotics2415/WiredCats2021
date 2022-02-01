package frc.auto;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class MultiPath extends SequentialCommandGroup {

    public MultiPath() {
        final PathPlannerTrajectory trajectory = PathPlanner.loadPath("MultiPathTestPt1", 3, 3, false);
        final PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("MultiPathTestPt2", 3, 3, false);
        // setting up the pid controllers (theta for rotation) (these pid values need
        // tuning prob)
        ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0, Constants.THETA_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        PIDController xController = new PIDController(0.5, 0, 0);
        PIDController yController = new PIDController(0.5, 0, 0);

        SequentialCommandGroup swerveControllerCommand = new PPSwerveControllerCommand(trajectory,
                Robot.swerveDrive::getPose,
                Constants.swerveKinematics, xController, yController, thetaController,
                Robot.swerveDrive::setModuleStates,
                Robot.swerveDrive).andThen(new InstantCommand(() -> Robot.swerveDrive.drive(0, 0, 0)));
        SequentialCommandGroup swerveControllerCommand2 = new PPSwerveControllerCommand(trajectory2,
                Robot.swerveDrive::getPose,
                Constants.swerveKinematics, xController, yController, thetaController,
                Robot.swerveDrive::setModuleStates,
                Robot.swerveDrive).andThen(new InstantCommand(() -> Robot.swerveDrive.drive(0, 0, 0)));
        addCommands(new InstantCommand(() -> Robot.swerveDrive.resetOdometry(trajectory.getInitialPose())),
                new ParallelCommandGroup(swerveControllerCommand,
                        new WaitCommand(trajectory.getTotalTimeSeconds() - 1)
                                .andThen(Robot.spindexer.getSpindexerCommand(), Robot.intake.getIntakeCommand())),
                new WaitCommand(1),
                new ParallelCommandGroup(
                        new ParallelCommandGroup(Robot.turret.getShooterCommand(), Robot.feeder.getFeederCommand()),
                        Robot.intake.getIntakeCommand()),
                Robot.spindexer.getSpindexerCommand().alongWith(
                        Robot.feeder.getFeederCommand()),
                swerveControllerCommand2);
    }
}
