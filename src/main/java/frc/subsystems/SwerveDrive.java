package frc.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveDrive {

    public SwerveDriveOdometry odometry;
    public SwerveModule[] swerveModules;
    public AHRS navX;

    public SwerveDrive() {
        SwerveModule[] swerveModules = {
                new SwerveModule(Constants.SwerveModuleName.FRONT_LEFT),
                new SwerveModule(Constants.SwerveModuleName.FRONT_RIGHT),
                new SwerveModule(Constants.SwerveModuleName.BACK_LEFT),
                new SwerveModule(Constants.SwerveModuleName.BACK_RIGHT)
        };
        navX = new AHRS(Port.kMXP);
        navX.zeroYaw();
        odometry = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw());
    }

    public void drive(double x, double y, double r) {
        x = (Math.abs(x) < Constants.DEADBAND) ? 0 : x;
        y = (Math.abs(y) < Constants.DEADBAND) ? 0 : y;
        r = (Math.abs(r) < Constants.DEADBAND) ? 0 : r;
        Translation2d translation = new Translation2d(y, x).times(Constants.MAX_SWERVE_SPEED);
        double rotation = r * Constants.MAX_ROTATION_SPEED;
        //assuming driver in control if reading off controller
        drive(translation, rotation, true, true);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented, boolean openLoop) {
        SwerveModuleState[] states = Constants.swerveKinematics.toSwerveModuleStates(fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_SWERVE_SPEED);

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNewState(states[i], openLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] states) {
        //only used in auto, so openloop false
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNewState(states[i], false);
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public void syncAzimuth() {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].syncAzimuth();
        }
    }

    public void printEncoders() {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].printAzimuthEncoder();
            swerveModules[i].printTalonEncoder();
        }
    }

    public void zeroYaw() {
        navX.zeroYaw();
    }

    public Rotation2d getYaw() {
        double yaw = navX.getYaw() + 180;
        return Rotation2d.fromDegrees(Constants.NAVX_FACING_UP ? yaw : 360 - yaw);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getYaw());
    }

}
