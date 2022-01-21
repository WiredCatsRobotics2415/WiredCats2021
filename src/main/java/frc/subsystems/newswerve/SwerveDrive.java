package frc.subsystems.newswerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveDrive {

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
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented, boolean openLoop) {
        SwerveModuleState[] states = Constants.swerveKinematics.toSwerveModuleStates(fieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.MAX_SWERVE_SPEED);
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setNewState(states[i], openLoop);
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public void zeroYaw() {
        navX.zeroYaw();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(Constants.NAVX_FACING_UP ? navX.getYaw() : navX.getYaw() * -1);
    }

}
