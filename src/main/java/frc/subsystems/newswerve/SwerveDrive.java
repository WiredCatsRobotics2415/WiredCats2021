package frc.subsystems.newswerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
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

    public void zeroYaw() {
        navX.zeroYaw();
    }
    
}
